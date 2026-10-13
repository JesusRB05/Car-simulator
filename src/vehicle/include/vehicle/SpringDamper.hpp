// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Piecewise-linear damper force-velocity curve, asymmetric in bump and rebound.
// The table carries any blow-off: a slope change between two speeds flattens the
// force growth beyond it. The force is linearly interpolated and clamped outside
// the tabulated range.
struct DamperTable {
    std::vector<Real> speeds; // m/s, strictly increasing
    std::vector<Real> forces; // N

    bool empty() const { return speeds.empty(); }

    // Force at a given shaft velocity (m/s, + = compression).
    Real forceAt(Real speed) const;
};

// Coil spring, damper and the progressive bump and rebound stops of one corner
// (PHYSICS_PLAN Section 5.2). Lengths are along the element (spring or damper),
// in metres; sign of force is positive in compression (the element resists being
// shortened).
struct SpringDamperCoefficients {
    // Coil spring: F = k (l0 - l) + preload.
    Real stiffness = 0.0;  // k (N/m)
    Real restLength = 0.0; // l0 (m)
    Real preload = 0.0;    // F_preload (N)

    // Damper: linear and asymmetric unless a table is supplied, which overrides
    // the two coefficients.
    Real dampingCompression = 0.0; // c, bump (N*s/m)
    Real dampingRebound = 0.0;     // c, rebound (N*s/m)
    DamperTable damperTable;

    // Progressive bump stop, engaged when the element is shorter than l_b.
    bool hasBumpStop = false;
    Real bumpStopLength = 0.0;    // l_b (m)
    Real bumpStopStiffness = 0.0; // k_b (N/m^n)
    Real bumpStopDamping = 0.0;   // c_b (N*s/m)
    Real bumpStopExponent = 2.0;  // n >= 1

    // Progressive rebound stop, engaged when the element is longer than l_r.
    bool hasReboundStop = false;
    Real reboundStopLength = 0.0;    // l_r (m)
    Real reboundStopStiffness = 0.0; // k_r (N/m^n)
    Real reboundStopDamping = 0.0;   // c_r (N*s/m)
    Real reboundStopExponent = 2.0;  // n >= 1
};

// State of the element for one step. The velocity is the compression rate (the
// rate at which the element shortens), positive in bump.
struct SpringDamperState {
    Real length = 0.0;   // l (m)
    Real velocity = 0.0; // compression rate (m/s), + = shortening
};

// The four contributions, kept apart so they can be validated individually.
struct SpringDamperForces {
    Real spring = 0.0;      // N
    Real damper = 0.0;      // N
    Real bumpStop = 0.0;    // N
    Real reboundStop = 0.0; // N

    Real total() const { return spring + damper + bumpStop + reboundStop; }
};

SpringDamperForces computeSpringDamperForces(const SpringDamperCoefficients& coefficients,
                                             const SpringDamperState& state);

} // namespace sim::vehicle
