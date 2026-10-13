// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Friction clutch (PHYSICS_PLAN Section 7.2). The clamp force comes from the
// pedal engagement; the torque capacity follows the uniform-pressure effective
// radius. When the two sides turn at a different speed the clutch transmits its
// capacity against the slip; when they are locked it transmits whatever the
// driveline needs, clamped to the capacity (and starts to slip if the clamp is
// hit).
struct FrictionClutch {
    Real frictionCoefficient = 0.30; // mu_c
    Real maxClampForce = 5000.0;     // F_n at full engagement (N)
    int frictionFaces = 2;           // n_f
    Real outerRadius = 0.11;         // R_o (m)
    Real innerRadius = 0.07;         // R_i (m)
    Real engagement = 1.0;           // [0, 1], 1 = fully clamped

    Real slipSpeed = 0.1;     // rad/s, smoothing scale of the slip sign
    Real lockThreshold = 0.5; // rad/s, below this the clutch is treated as locked

    Real clampForce() const;      // F_n * engagement (N)
    Real effectiveRadius() const; // R_eff (m)
    Real capacity() const;        // T_c (N*m)
};

struct ClutchTorque {
    Real torque = 0.0; // N*m, + = drives the output forward
    bool slipping = false;
};

// Torque transmitted for a given slip (omega_in - omega_out) and, when locked, the
// torque the driveline would need.
ClutchTorque computeClutchTorque(const FrictionClutch& clutch, Real slipSpeed, Real requiredTorque);

// Centrifugal clutch capacity (karting), T_c = k_cf (omega_e^2 - omega_engage^2)+.
Real centrifugalClutchCapacity(Real coefficient, Real engineAngularVelocity, Real engageAngularVelocity);

} // namespace sim::vehicle
