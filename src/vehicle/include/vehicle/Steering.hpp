// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Steering geometry and column torque (PHYSICS_PLAN Section 5.5). The road-wheel
// angles come from the rack ratio and from how much of the Ackermann condition
// the tie-rod geometry produces; the column torque is the physical
// force-feedback signal, never a spring to centre.
struct AckermannGeometry {
    Real track = 1.5;     // t (m), front axle
    Real wheelbase = 2.6; // L (m)
};

// Full Ackermann outer angle for a given inner angle:
// cot(delta_o) - cot(delta_i) = t / L.
Real ackermannOuterAngle(const AckermannGeometry& geometry, Real innerAngle);

struct SteerAngles {
    Real inner = 0.0; // rad
    Real outer = 0.0; // rad
};

// Inner/outer angles at a given percentage of Ackermann (0 = parallel steer,
// 1 = full Ackermann).
SteerAngles steerAngles(const AckermannGeometry& geometry, Real innerAngle, Real ackermannPercent);

// Rack ratio: road-wheel angle from the steering-wheel angle.
Real roadWheelAngle(Real steeringRatio, Real steeringWheelAngle);

// One road wheel's contribution to the aligning torque about the steering axis.
struct WheelSteerLoad {
    Real aligningMoment = 0.0;  // M_z (N*m)
    Real lateralForce = 0.0;    // F_y (N)
    Real mechanicalTrail = 0.0; // t_m (m)
    Real pneumaticTrail = 0.0;  // t_p (m)
};

// Steering-column torque: the algebraic sum of the road-wheel aligning torques
// divided by the steering ratio efficiency, plus the assist and friction torques
// supplied by the caller.
Real steeringColumnTorque(const std::vector<WheelSteerLoad>& wheels, Real ratioEfficiency, Real assistTorque,
                          Real frictionTorque);

} // namespace sim::vehicle
