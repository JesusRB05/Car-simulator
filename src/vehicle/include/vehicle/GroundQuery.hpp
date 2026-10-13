// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;
using physics::Vec3;

// What a wheel sees where it touches the ground. Mirrors the surface data the
// raycast suspension needs without depending on any particular terrain class.
struct GroundSample {
    Real height = 0.0;              // y at (x, z) (m)
    Vec3 normal{0.0, 1.0, 0.0};     // unit surface normal (+Y up)
    Real friction = 1.0;            // peak tyre/road friction scale
    Real rollingResistance = 0.015; // Crr (force / load)
};

// Analytic ground the raycast wheels query. The vehicle model only needs a
// height field with a normal and a friction scale, so the application's track
// and a flat test plane can both satisfy it. Kept in the vehicle library so the
// model does not depend on the app.
class IGroundQuery {
public:
    virtual ~IGroundQuery() = default;

    // Height, normal and surface properties at the horizontal position (x, z).
    virtual GroundSample sample(Real x, Real z) const = 0;
};

// An infinite flat plane at a fixed height, for tests and the simple scenes.
class FlatGroundQuery final : public IGroundQuery {
public:
    explicit FlatGroundQuery(Real planeHeight = 0.0, Real friction = 1.0, Real rollingResistance = 0.015)
        : height_(planeHeight), friction_(friction), rollingResistance_(rollingResistance) {}

    GroundSample sample(Real, Real) const override {
        GroundSample result;
        result.height = height_;
        result.friction = friction_;
        result.rollingResistance = rollingResistance_;
        return result;
    }

private:
    Real height_ = 0.0;
    Real friction_ = 1.0;
    Real rollingResistance_ = 0.015;
};

} // namespace sim::vehicle
