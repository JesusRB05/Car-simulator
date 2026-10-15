// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// AABB-tree broadphase (Phase 1, P1.1). It replaces the brute-force O(n^2) sweep
// with a bounding volume hierarchy built once per detection pass: the bodies are
// sorted into a median-split tree over their world AABBs and the tree is walked
// against itself to report every overlapping leaf pair. The split axis and the
// tie-break on the body index make the tree, and therefore the pair list, fully
// reproducible, which the deterministic step relies on.

#include <cstddef>
#include <vector>

#include "physics/Collision.hpp"

namespace sim::physics {

// A deterministic bounding volume hierarchy over a batch of world-space AABBs.
// `build` is given the AABBs of every body plus the subset of indices that take
// part (the broadphase narrows to collidable bodies); `overlappingPairs` then
// reports each overlapping pair once, ordered with `bodyA < bodyB`.
class AabbTree {
public:
    void build(const std::vector<Aabb>& boxes, const std::vector<std::size_t>& indices);

    // Every pair of leaves in the tree whose AABBs overlap, sorted and deduplicated.
    // `boxes` must be the same array passed to build().
    std::vector<BodyPair> overlappingPairs(const std::vector<Aabb>& boxes) const;

    // Appends the body index of every leaf whose cached AABB overlaps `box`. Used
    // to test a handful of moving bodies against a tree of static geometry that
    // was built once, without rebuilding it every step.
    void query(const Aabb& box, std::vector<std::size_t>& out) const;

    bool empty() const { return nodes_.empty(); }

private:
    struct Node {
        Aabb bounds;
        // Leaf when left < 0; the leaf then owns order_[begin, end).
        int left = -1;
        int right = -1;
        std::size_t begin = 0;
        std::size_t end = 0;
    };

    int buildNode(const std::vector<Aabb>& boxes, std::size_t begin, std::size_t end);
    void collideNodes(int nodeA, int nodeB, const std::vector<Aabb>& boxes, std::vector<BodyPair>& out) const;
    void queryNode(int node, const Aabb& box, std::vector<std::size_t>& out) const;

    std::vector<Node> nodes_;
    std::vector<std::size_t> order_; // body indices, permuted by the build
};

} // namespace sim::physics
