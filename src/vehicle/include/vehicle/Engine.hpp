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

// Engine dynamics (PHYSICS_PLAN Section 7.1): the crankshaft inertia integrated
// from the map torque minus the friction and the load, with a first-order
// throttle lag, a rev limiter by fuel cut with hysteresis and a PI idle
// controller.
struct EngineConfig {
    Real inertia = 0.5;               // J_e (kg*m^2)
    Real throttleTimeConstant = 0.05; // tau_theta (s)
    Real revLimit = 700.0;            // rad/s
    Real revLimitHysteresis = 30.0;   // rad/s
    Real idleSpeed = 90.0;            // rad/s
    Real idleProportionalGain = 1.0;  // N*m/(rad/s)
    Real idleIntegralGain = 2.0;      // N*m/(rad*s)
    Real idleThrottleThreshold = 0.02;
};

class Engine {
public:
    EngineConfig config;
    EngineTorqueMap map;
    EngineFriction friction;

    void reset();

    // One fixed step: advances the throttle lag, applies the rev limiter and the
    // idle controller, integrates the crankshaft and returns the torque at the
    // crankshaft (N*m).
    Real step(Real deltaSeconds, Real throttle, Real loadTorque);

    Real angularVelocity() const { return angularVelocity_; }
    Real effectiveThrottle() const { return effectiveThrottle_; }
    bool fuelCut() const { return fuelCut_; }

private:
    Real angularVelocity_ = 0.0;
    Real effectiveThrottle_ = 0.0;
    Real idleIntegral_ = 0.0;
    bool fuelCut_ = false;
};

} // namespace sim::vehicle
