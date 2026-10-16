// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "drive/Track.hpp"
#include "drive/TrackCollisionMesh.hpp"
#include "vehicle/GroundQuery.hpp"

namespace sim::app::drive {

// Adapts the app's procedural circuit to the vehicle library's ground query, so
// a VehicleModel can drive on the Track without the library depending on the
// app. The Track already answers height/normal/friction/rolling-resistance; this
// only re-wraps that answer in the library's GroundSample.
//
// This is the *analytic* adapter (the exact height field). The drive scene uses
// TrackMeshGroundQuery, which reads the geometry from the collision mesh.
class TrackGroundQuery final : public vehicle::IGroundQuery {
public:
    explicit TrackGroundQuery(const Track& track) : track_(&track) {}

    vehicle::GroundSample sample(Real x, Real z) const override {
        const GroundSample surface = track_->sample(x, z);
        vehicle::GroundSample result;
        result.height = surface.height;
        result.normal = surface.normal;
        result.friction = surface.friction;
        result.rollingResistance = surface.rollingResistance;
        return result;
    }

private:
    const Track* track_;
};

// Ground query backed by the track's collision triangle mesh: the height and
// normal come from a vertical raycast against the per-triangle BVH, while the
// material (friction, rolling resistance, surface type) comes from the Track.
// This is what the drive scene feeds the wheels, so they query real mesh
// geometry instead of the analytic height field. Outside the mesh it falls back
// to the analytic surface, so a car that leaves the mapped area is still
// supported.
class TrackMeshGroundQuery final : public vehicle::IGroundQuery {
public:
    TrackMeshGroundQuery(const Track& track, const TrackCollisionMesh& mesh) : track_(&track), mesh_(&mesh) {}

    vehicle::GroundSample sample(Real x, Real z) const override {
        const GroundSample material = track_->sample(x, z);
        vehicle::GroundSample result;
        result.friction = material.friction;
        result.rollingResistance = material.rollingResistance;

        Real height = 0.0;
        Vec3 normal(0.0, 1.0, 0.0);
        if (mesh_->sample(x, z, height, normal)) {
            result.height = height;
            result.normal = normal;
        } else {
            result.height = material.height;
            result.normal = material.normal;
        }
        return result;
    }

private:
    const Track* track_;
    const TrackCollisionMesh* mesh_;
};

} // namespace sim::app::drive
