// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "drive/Track.hpp"
#include "vehicle/GroundQuery.hpp"

namespace sim::app::drive {

// Adapts the app's procedural circuit to the vehicle library's ground query, so
// a VehicleModel can drive on the Track without the library depending on the
// app. The Track already answers height/normal/friction/rolling-resistance; this
// only re-wraps that answer in the library's GroundSample.
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

} // namespace sim::app::drive
