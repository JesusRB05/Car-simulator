// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <vector>

#include "physics/Collision.hpp"
#include "physics/TriangleMesh.hpp"

namespace sim::physics {

// The closest intersection of a ray with a TriangleMeshBvh.
struct MeshRayHit {
    bool hit = false;
    Real distance = 0.0;        // in units of the query direction's length
    Vec3 point{0.0};            // origin + distance * direction
    Vec3 normal{0.0, 1.0, 0.0}; // unit, facing the incoming ray
    std::size_t triangle = 0;   // index into the mesh
};

// A bounding volume hierarchy over the triangles of a TriangleMesh, one leaf per
// triangle. This is the per-primitive acceleration structure a triangle mesh
// needs so a ray does not have to test every triangle: the static geometry is
// built once and queried many times (a raycast wheel does one query per corner
// per step).
//
// The tree is a median split on the widest axis, with the triangle index
// breaking centre ties, so it is fully reproducible (the deterministic step may
// depend on the order in which ties are resolved).
//
// Vertices are stored in the mesh's own (local) frame, so the query ray must be
// expressed in that same frame. For a static track mesh built in world space the
// two coincide.
class TriangleMeshBvh {
public:
    // Builds the hierarchy and keeps the mesh alive inside the BVH.
    void build(TriangleMesh mesh);

    bool empty() const { return order_.empty(); }
    std::size_t triangleCount() const { return order_.size(); }
    std::size_t nodeCount() const { return nodes_.size(); }
    std::size_t maxDepth() const { return maxDepth_; }
    const TriangleMesh& mesh() const { return mesh_; }

    // Closest intersection with the ray `origin + t * direction`, for t in
    // [0, maxDistance]. `direction` need not be unit length (a non-unit
    // direction scales `distance`); a zero-length direction or an empty mesh
    // reports no hit. The hit normal faces the incoming ray, so a downward query
    // against the top of a surface returns the upward normal regardless of the
    // triangle winding.
    MeshRayHit raycast(const Vec3& origin, const Vec3& direction, Real maxDistance) const;

private:
    struct Node {
        Aabb bounds;
        int left = -1; // < 0 for a leaf
        int right = -1;
        std::size_t begin = 0; // leaf triangle range in `order_`
        std::size_t end = 0;
    };

    int buildNode(std::size_t begin, std::size_t end, std::size_t depth);
    void raycastNode(int node, const Vec3& origin, const Vec3& direction, Real maxDistance, MeshRayHit& hit) const;

    TriangleMesh mesh_;
    std::vector<Aabb> bounds_; // one AABB per triangle
    std::vector<std::size_t> order_;
    std::vector<Node> nodes_;
    std::size_t maxDepth_ = 0;
};

} // namespace sim::physics
