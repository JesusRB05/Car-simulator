// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Differential (PHYSICS_PLAN Section 7.4). Every type splits the input torque
// between the two outputs; the open, clutch-pack, Torsen and viscous types share
// the kinematic law 2 w_in = w_L + w_R, the spool forces w_L = w_R (modelled here
// as a very stiff viscous coupling).
enum class DifferentialType {
    Open,
    Locked,
    ClutchPack,
    Torsen,
    Viscous,
};

struct DifferentialCoefficients {
    DifferentialType type = DifferentialType::Open;
    Real preloadTorque = 0.0;      // T_pre (N*m)
    Real powerRamp = 0.0;          // k_p (dimensionless)
    Real coastRamp = 0.0;          // k_c (dimensionless)
    Real slipScale = 0.1;          // omega_epsilon (rad/s)
    Real torqueBiasRatio = 2.0;    // TBR (Torsen)
    Real viscousCoefficient = 0.0; // c_v (N*m*s/rad)
    Real lockStiffness = 1.0e5;    // spool coupling (N*m*s/rad)
};

struct DifferentialInput {
    Real inputTorque = 0.0; // N*m at the ring gear
    Real leftAngularVelocity = 0.0;
    Real rightAngularVelocity = 0.0;
};

struct DifferentialOutput {
    Real leftTorque = 0.0;  // N*m
    Real rightTorque = 0.0; // N*m
};

DifferentialOutput computeDifferentialTorques(const DifferentialCoefficients& coefficients,
                                              const DifferentialInput& input);

} // namespace sim::vehicle
