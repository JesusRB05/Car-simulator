// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "drive/Track.hpp"
#include "physics/TriangleMesh.hpp"
#include "physics/TriangleMeshBvh.hpp"

namespace sim::app::drive {

// The drivable surface of a Track as a triangle mesh: the asphalt ribbon, the
// kerbs and a terrain grid, built once and indexed by a per-triangle BVH so a
// wheel ray is cheap. The wheels query this mesh (through TrackMeshGroundQuery)
// instead of the analytic height field, which is the step towards circuits whose
// surface is real mesh geometry.
//
// The mesh is built from the same road points the renderer draws, so the wheels
// sit on the surface that is actually visible. Vertices are in world space (the
// track is static), so a query ray needs no transform.
class TrackCollisionMesh {
public:
    explicit TrackCollisionMesh(const Track& track);

    // Height and unit surface normal (+Y up) under (x, z), from a vertical
    // raycast, or false when the ray misses the mesh (outside its extent).
    bool sample(Real x, Real z, Real& height, Vec3& normal) const;

    const physics::TriangleMesh& mesh() const { return bvh_.mesh(); }
    const physics::TriangleMeshBvh& bvh() const { return bvh_; }

private:
    physics::TriangleMeshBvh bvh_;
};

} // namespace sim::app::drive
