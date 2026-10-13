// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// The tyre as a nonlinear, one-sided vertical spring-damper, plus the effective
// rolling radius (Phase 2, P2.1; PHYSICS_PLAN Section 4.2). Deflection is
// measured along the effective contact normal; the tyre never pulls, so an
// airborne corner carries zero load.
struct TireVerticalCoefficients {
    Real unloadedRadius = 0.31; // R0 (m)
    Real kz0 = 200000.0;        // linear vertical stiffness (N/m)
    Real kz1 = 0.0;             // stiffening with deflection (N/m^2)
    Real cz = 1500.0;           // vertical damping (N*s/m)
    Real nominalLoad = 4000.0;  // Fz0 (N), normalising the radius curve

    // Effective rolling radius Re(Fz, Omega) shape (Pacejka B, D, F, c_Omega).
    Real radiusB = 10.0;
    Real radiusD = 0.3;
    Real radiusF = 0.03;
    Real radiusOmega = 0.0;
};

// Inputs at one corner for this step.
struct TireVerticalState {
    Real deflection = 0.0;     // delta (m), >= 0 when touching
    Real deflectionRate = 0.0; // d(delta)/dt (m/s)
    Real spin = 0.0;           // wheel spin Omega (rad/s)
};

struct TireVerticalResult {
    Real normalLoad = 0.0;        // Fz (N), >= 0
    Real loadedRadius = 0.0;      // Rl = R0 - delta (m)
    Real effectiveRadius = 0.0;   // Re (m)
    Real contactHalfLength = 0.0; // a (m)
};

TireVerticalResult evaluateTireVertical(const TireVerticalCoefficients& coefficients, const TireVerticalState& state);

} // namespace sim::vehicle
