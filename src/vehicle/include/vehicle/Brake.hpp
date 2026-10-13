// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Disc brake (PHYSICS_PLAN Section 7.6). The torque grows with the line pressure
// above a threshold and with the pad friction, the piston area and the effective
// radius; the friction coefficient may be temperature dependent through a map.
struct DiscBrake {
    Real padFrictionCoefficient = 0.40; // mu_p
    Real pistonArea = 0.002;            // A_p (m^2)
    Real effectiveRadius = 0.13;        // R_eff (m)
    Real thresholdPressure = 0.0;       // p_thr (Pa)

    // T_b = 2 mu_p A_p (p - p_thr)+ R_eff (N*m); no drag below the threshold.
    Real torque(Real linePressure) const;
};

// Front/rear split from the master cylinders; the handbrake adds to the rear only.
struct BrakeBias {
    Real frontFraction = 0.6;     // [0, 1]
    Real handbrakePressure = 0.0; // Pa added to the rear line
};

Real frontLinePressure(Real masterPressure, const BrakeBias& bias);
Real rearLinePressure(Real masterPressure, const BrakeBias& bias);

// Thermal state of a disc: the heat from the braking power against convection and
// (optionally) radiation.
struct BrakeThermal {
    Real discMass = 5.0;               // m_d (kg)
    Real specificHeat = 500.0;         // c_p (J/(kg*K))
    Real coolingArea = 0.1;            // A (m^2)
    Real convectionCoefficient = 50.0; // h (W/(m^2*K))
    Real emissivity = 0.0;             // 0 disables radiation
    Real ambientTemperature = 293.15;  // K
};

// Advances the disc temperature one step (explicit) and returns it (K).
Real stepBrakeTemperature(const BrakeThermal& thermal, Real temperature, Real brakeTorque, Real wheelAngularVelocity,
                          Real deltaSeconds);

} // namespace sim::vehicle
