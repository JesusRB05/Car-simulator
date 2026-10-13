// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"
#include "vehicle/ITireModel.hpp"

namespace sim::vehicle {

using physics::Real;

// Implicit longitudinal wheel-spin integration (Phase 2, P2.1; PHYSICS_PLAN
// Section 4.8). The tyre's longitudinal force depends steeply on the spin near
// lock-up, where an explicit step at 1 kHz oscillates or diverges, so the wheel
// equation J dOmega/dt = T_drive - T_brake - Re Fx - My is solved with a
// linearised implicit step. The caller owns the spin, keeping the model pure.
struct WheelSpinCoefficients {
    Real inertia = 1.0;           // J_w (kg*m^2)
    Real minContactSpeed = 0.1;   // V_min (m/s), guards the slip-ratio division
    Real derivativeStep = 1.0e-4; // finite-difference step for dFx/dOmega (rad/s)
};

struct WheelSpinInput {
    Real driveTorque = 0.0;             // T_drive (N*m)
    Real brakeTorque = 0.0;             // T_brake (N*m), >= 0, opposes the spin
    Real contactSpeed = 0.0;            // V_cx (m/s), wheel-centre longitudinal speed
    Real effectiveRadius = 0.31;        // R_e (m)
    Real rollingResistanceMoment = 0.0; // M_y (N*m)
};

// Wheel spin state owned and advanced by the caller.
struct WheelSpinState {
    Real spin = 0.0;     // Omega (rad/s)
    bool locked = false; // true when the brake holds the wheel at zero spin
};

struct WheelSpinResult {
    Real spin = 0.0;              // Omega_{n+1} (rad/s)
    Real slipRatio = 0.0;         // kappa at the new spin
    Real longitudinalForce = 0.0; // Fx at the new spin (N)
    bool locked = false;
};

// Advances the spin by dt and returns the new state. `context` supplies the
// non-longitudinal tyre state (normal load, slip angle, camber, friction,
// contact speed); its slipRatio field is ignored and recomputed from the spin.
// If a braking torque would reverse the wheel through zero, the spin is clamped
// to zero and the wheel is reported locked (static friction).
WheelSpinResult stepWheelSpin(const WheelSpinCoefficients& coefficients, const ITireModel& tire,
                              const TireState& context, const WheelSpinInput& input, Real dt, WheelSpinState& state);

} // namespace sim::vehicle
