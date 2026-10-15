// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Broadphase.hpp"

#include <algorithm>

namespace sim::physics {
namespace {

Real component(const Vec3& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

Aabb joined(const Aabb& a, const Aabb& b) {
    return Aabb{glm::min(a.min, b.min), glm::max(a.max, b.max)};
}

} // namespace

void AabbTree::build(const std::vector<Aabb>& boxes, const std::vector<std::size_t>& indices) {
    nodes_.clear();
    order_ = indices;
    if (order_.empty()) {
        return;
    }
    buildNode(boxes, 0, order_.size());
}

int AabbTree::buildNode(const std::vector<Aabb>& boxes, std::size_t begin, std::size_t end) {
    const int self = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{});

    Aabb bounds = boxes[order_[begin]];
    for (std::size_t i = begin + 1; i < end; ++i) {
        bounds = joined(bounds, boxes[order_[i]]);
    }

    if (end - begin == 1) {
        nodes_[self].bounds = bounds;
        nodes_[self].begin = begin;
        nodes_[self].end = end;
        nodes_[self].left = -1;
        nodes_[self].right = -1;
        return self;
    }

    // Median split along the widest axis of the bounds. The comparator is a
    // strict total order (the body index breaks centre ties), so partitioning
    // around the median is unique and the tree is reproducible.
    const Vec3 extent = bounds.extent();
    int axis = 0;
    if (extent.y > extent.x && extent.y >= extent.z) {
        axis = 1;
    } else if (extent.z > extent.x && extent.z > extent.y) {
        axis = 2;
    }
    const std::size_t mid = begin + (end - begin) / 2;
    std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                     [&](std::size_t lhs, std::size_t rhs) {
                         const Real cl = component(boxes[lhs].center(), axis);
                         const Real cr = component(boxes[rhs].center(), axis);
                         if (cl != cr) {
                             return cl < cr;
                         }
                         return lhs < rhs;
                     });

    const int left = buildNode(boxes, begin, mid);
    const int right = buildNode(boxes, mid, end);

    nodes_[self].bounds = bounds;
    nodes_[self].begin = begin;
    nodes_[self].end = end;
    nodes_[self].left = left;
    nodes_[self].right = right;
    return self;
}

void AabbTree::collideNodes(int nodeA, int nodeB, const std::vector<Aabb>& boxes, std::vector<BodyPair>& out) const {
    const Node& a = nodes_[nodeA];
    const Node& b = nodes_[nodeB];
    if (!a.bounds.overlaps(b.bounds)) {
        return;
    }
    const bool aLeaf = a.left < 0;
    const bool bLeaf = b.left < 0;
    if (aLeaf && bLeaf) {
        for (std::size_t i = a.begin; i < a.end; ++i) {
            for (std::size_t j = b.begin; j < b.end; ++j) {
                const std::size_t bodyA = order_[i];
                const std::size_t bodyB = order_[j];
                if (bodyA < bodyB) {
                    out.push_back(BodyPair{bodyA, bodyB});
                }
            }
        }
        return;
    }
    if (bLeaf) {
        collideNodes(a.left, nodeB, boxes, out);
        collideNodes(a.right, nodeB, boxes, out);
        return;
    }
    if (aLeaf) {
        collideNodes(nodeA, b.left, boxes, out);
        collideNodes(nodeA, b.right, boxes, out);
        return;
    }
    // Both internal: descend the side with more bodies so the recursion shrinks
    // quickly and the unordered pair (i, j) is only ever visited with i < j.
    if (a.end - a.begin >= b.end - b.begin) {
        collideNodes(a.left, nodeB, boxes, out);
        collideNodes(a.right, nodeB, boxes, out);
    } else {
        collideNodes(nodeA, b.left, boxes, out);
        collideNodes(nodeA, b.right, boxes, out);
    }
}

void AabbTree::queryNode(int node, const Aabb& box, std::vector<std::size_t>& out) const {
    const Node& current = nodes_[node];
    if (!current.bounds.overlaps(box)) {
        return;
    }
    if (current.left < 0) {
        for (std::size_t i = current.begin; i < current.end; ++i) {
            out.push_back(order_[i]);
        }
        return;
    }
    queryNode(current.left, box, out);
    queryNode(current.right, box, out);
}

void AabbTree::query(const Aabb& box, std::vector<std::size_t>& out) const {
    if (nodes_.empty()) {
        return;
    }
    queryNode(0, box, out);
}

std::vector<BodyPair> AabbTree::overlappingPairs(const std::vector<Aabb>& boxes) const {
    std::vector<BodyPair> pairs;
    if (nodes_.empty()) {
        return pairs;
    }
    collideNodes(0, 0, boxes, pairs);
    std::sort(pairs.begin(), pairs.end(), [](const BodyPair& lhs, const BodyPair& rhs) {
        if (lhs.bodyA != rhs.bodyA) {
            return lhs.bodyA < rhs.bodyA;
        }
        return lhs.bodyB < rhs.bodyB;
    });
    pairs.erase(std::unique(pairs.begin(), pairs.end(),
                            [](const BodyPair& lhs, const BodyPair& rhs) {
                                return lhs.bodyA == rhs.bodyA && lhs.bodyB == rhs.bodyB;
                            }),
                pairs.end());
    return pairs;
}

} // namespace sim::physics
