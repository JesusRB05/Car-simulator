// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/TriangleMeshBvh.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim::physics {
namespace {

Real component(const Vec3& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

Aabb joined(const Aabb& a, const Aabb& b) {
    return Aabb{glm::min(a.min, b.min), glm::max(a.max, b.max)};
}

// Robust slab test (Barnes): `std::fmin`/`std::fmax` ignore the NaN a zero
// direction component produces at an axis-aligned boundary, so a ray parallel
// to a slab is still handled correctly. `maxDistance` bounds the far end.
bool slabHit(const Aabb& box, const Vec3& origin, const Vec3& direction, Real maxDistance) {
    Real tMin = 0.0;
    Real tMax = maxDistance;
    for (int axis = 0; axis < 3; ++axis) {
        const Real inverse = 1.0 / component(direction, axis); // IEEE: +-inf on a zero component
        const Real t1 = (component(box.min, axis) - component(origin, axis)) * inverse;
        const Real t2 = (component(box.max, axis) - component(origin, axis)) * inverse;
        tMin = std::fmax(tMin, std::fmin(t1, t2));
        tMax = std::fmin(tMax, std::fmax(t1, t2));
        if (tMax < tMin) {
            return false;
        }
    }
    return true;
}

// Moller-Trumbore. `direction` is not normalised; `t` is in units of its length.
bool intersectTriangle(const Vec3& origin, const Vec3& direction, const Vec3& a, const Vec3& b, const Vec3& c, Real& t,
                       Real& u, Real& v) {
    const Vec3 edge1 = b - a;
    const Vec3 edge2 = c - a;
    const Vec3 pvec = glm::cross(direction, edge2);
    const Real determinant = glm::dot(edge1, pvec);
    if (std::abs(determinant) < 1e-12) {
        return false; // ray parallel to the triangle, or a degenerate triangle
    }
    const Real inverseDeterminant = 1.0 / determinant;
    const Vec3 tvec = origin - a;
    u = glm::dot(tvec, pvec) * inverseDeterminant;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const Vec3 qvec = glm::cross(tvec, edge1);
    v = glm::dot(direction, qvec) * inverseDeterminant;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    t = glm::dot(edge2, qvec) * inverseDeterminant;
    return t >= 0.0;
}

} // namespace

void TriangleMeshBvh::build(TriangleMesh mesh) {
    mesh_ = std::move(mesh);
    bounds_.clear();
    order_.clear();
    nodes_.clear();
    maxDepth_ = 0;

    const std::size_t count = mesh_.valid() ? mesh_.triangleCount() : 0;
    if (count == 0) {
        mesh_ = TriangleMesh{};
        return;
    }

    bounds_.resize(count);
    order_.resize(count);
    for (std::size_t triangle = 0; triangle < count; ++triangle) {
        Vec3 a, b, c;
        mesh_.triangle(triangle, a, b, c);
        bounds_[triangle] = Aabb{glm::min(a, glm::min(b, c)), glm::max(a, glm::max(b, c))};
        order_[triangle] = triangle;
    }
    buildNode(0, count, 0);
}

int TriangleMeshBvh::buildNode(std::size_t begin, std::size_t end, std::size_t depth) {
    maxDepth_ = std::max(maxDepth_, depth);
    const int self = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{});

    Aabb bounds = bounds_[order_[begin]];
    for (std::size_t i = begin + 1; i < end; ++i) {
        bounds = joined(bounds, bounds_[order_[i]]);
    }

    if (end - begin == 1) {
        nodes_[self].bounds = bounds;
        nodes_[self].begin = begin;
        nodes_[self].end = end;
        return self;
    }

    // Median split along the widest axis; the triangle index breaks centre ties,
    // so the partition (and therefore the tree) is unique.
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
                         const Real left = component(bounds_[lhs].center(), axis);
                         const Real right = component(bounds_[rhs].center(), axis);
                         if (left != right) {
                             return left < right;
                         }
                         return lhs < rhs;
                     });

    const int left = buildNode(begin, mid, depth + 1);
    const int right = buildNode(mid, end, depth + 1);

    nodes_[self].bounds = bounds;
    nodes_[self].begin = begin;
    nodes_[self].end = end;
    nodes_[self].left = left;
    nodes_[self].right = right;
    return self;
}

MeshRayHit TriangleMeshBvh::raycast(const Vec3& origin, const Vec3& direction, Real maxDistance) const {
    MeshRayHit result;
    if (nodes_.empty() || maxDistance < 0.0 || glm::dot(direction, direction) <= 0.0) {
        return result;
    }
    raycastNode(0, origin, direction, maxDistance, result);
    return result;
}

void TriangleMeshBvh::raycastNode(int node, const Vec3& origin, const Vec3& direction, Real maxDistance,
                                  MeshRayHit& hit) const {
    const Node& current = nodes_[node];
    const Real limit = hit.hit ? std::min(maxDistance, hit.distance) : maxDistance;
    if (!slabHit(current.bounds, origin, direction, limit)) {
        return;
    }

    if (current.left < 0) {
        for (std::size_t i = current.begin; i < current.end; ++i) {
            const std::size_t triangle = order_[i];
            Vec3 a, b, c;
            mesh_.triangle(triangle, a, b, c);
            Real t = 0.0;
            Real u = 0.0;
            Real v = 0.0;
            if (!intersectTriangle(origin, direction, a, b, c, t, u, v)) {
                continue;
            }
            if (t > limit || (hit.hit && t >= hit.distance)) {
                continue;
            }
            hit.hit = true;
            hit.distance = t;
            hit.triangle = triangle;
            hit.point = origin + direction * t;
            Vec3 normal = glm::cross(b - a, c - a);
            const Real lengthSquared = glm::dot(normal, normal);
            if (lengthSquared > 0.0) {
                normal /= std::sqrt(lengthSquared);
            }
            if (glm::dot(normal, direction) > 0.0) {
                normal = -normal; // face the incoming ray
            }
            hit.normal = normal;
        }
        return;
    }

    raycastNode(current.left, origin, direction, maxDistance, hit);
    raycastNode(current.right, origin, direction, maxDistance, hit);
}

} // namespace sim::physics
