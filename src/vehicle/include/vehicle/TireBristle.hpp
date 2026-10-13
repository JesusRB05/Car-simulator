// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Bristle (LuGre) friction for standstill and very low speed (Phase 2, P2.4;
// PHYSICS_PLAN Section 4.5). Below a speed threshold the slip definitions of the
// steady-state model are singular, so a bristle model blends in. Its state is the
// mean bristle deflection z, integrated implicitly so it is stable at 1 kHz, and
// bounded by the static friction g(v) with the Stribeck decay. The caller blends
// the result with the steady-state Magic Formula by `blend` (1 at standstill, 0
// when moving); this is what keeps a parked car parked and removes low-speed
// creep. The state is owned by the caller because the tyre model is pure.
struct TireBristleCoefficients {
    Real sigma0 = 2.0e5;         // bristle stiffness (N/m)
    Real sigma1 = 200.0;         // bristle micro-damping (N*s/m)
    Real sigma2 = 0.0;           // viscous friction (N*s/m)
    Real staticFriction = 1.4;   // mu_s, the breakaway coefficient
    Real kineticFriction = 1.1;  // mu_k, the sliding coefficient
    Real stictionVelocity = 0.1; // vs (m/s), the Stribeck decay speed
    Real blendSpeed = 0.5;       // bristle weight falls from 1 at rest to 0 here (m/s)
};

// Bristle deflection per patch direction.
struct TireBristleState {
    Real longitudinal = 0.0; // z_x (m)
    Real lateral = 0.0;      // z_y (m)
};

struct TireBristleResult {
    Real longitudinal = 0.0; // bristle friction force Fx (N)
    Real lateral = 0.0;      // bristle friction force Fy (N)
    Real blend = 0.0;        // weight of the bristle model in [0, 1]
};

// Advances the bristle state by dt and returns its force. `vLong` and `vLat` are
// the contact-patch slip velocities (m/s); the returned friction force opposes
// them. A lifted tyre (normalLoad <= 0) resets the deflection and gives no force.
TireBristleResult updateTireBristle(const TireBristleCoefficients& coefficients, Real normalLoad, Real vLong, Real vLat,
                                    Real dt, TireBristleState& state);

} // namespace sim::vehicle
