// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;
using physics::Vec3;

// A rubber bushing at a pivot (PHYSICS_PLAN Section 5.4): a linear, per-axis
// spring in the six degrees of freedom. Given the relative displacement of the
// two frames it produces a restoring force and moment, which is what makes the
// suspension deflect under load (toe and camber compliance).
//
// The stiffness is diagonal in the pivot frame: one translational stiffness per
// axis (N/m) and one rotational stiffness per axis (N*m/rad).
struct BushingCoefficients {
    Vec3 translationalStiffness{0.0, 0.0, 0.0}; // N/m
    Vec3 rotationalStiffness{0.0, 0.0, 0.0};    // N*m/rad
};

// Relative displacement of the two frames at the pivot: translation (m) and a
// rotation vector (rad).
struct BushingState {
    Vec3 displacement{0.0, 0.0, 0.0}; // m
    Vec3 rotation{0.0, 0.0, 0.0};     // rad
};

// Restoring force and moment applied to the moving frame.
struct BushingForces {
    Vec3 force{0.0, 0.0, 0.0};  // N
    Vec3 moment{0.0, 0.0, 0.0}; // N*m
};

BushingForces computeBushingForces(const BushingCoefficients& coefficients, const BushingState& state);

} // namespace sim::vehicle
