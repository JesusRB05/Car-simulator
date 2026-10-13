// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Engine torque map (PHYSICS_PLAN Section 7.1). T_e(w, theta) is bilinearly
// interpolated on a grid of engine speed (rad/s) and throttle [0, 1]; the closed
// throttle column is negative, which is the engine braking.
//
// torque is stored row-major over the speed axis: torque[i * throttle.size() + j]
// is the value at speed[i] and throttle[j].
struct EngineTorqueMap {
    std::vector<Real> speed;    // rad/s, increasing
    std::vector<Real> throttle; // [0, 1], increasing
    std::vector<Real> torque;   // N*m, size speed.size() * throttle.size()

    bool empty() const;
    Real torqueAt(Real engineSpeed, Real throttle) const;
};

// Engine friction (and pumping) torque, T_f = a + b*w + c*w^2 (N*m).
struct EngineFriction {
    Real constant = 0.0;  // a (N*m)
    Real linear = 0.0;    // b (N*m*s/rad)
    Real quadratic = 0.0; // c (N*m*s^2/rad^2)

    Real torqueAt(Real angularVelocity) const;
};

} // namespace sim::vehicle
