// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Driveline compliance (PHYSICS_PLAN Section 7.5). A half-shaft or propshaft is a
// torsional spring-damper with lash: within the backlash the two ends turn freely
// and transmit nothing, outside it the twist springs up and, with the damping,
// gives the wind-up and the clunk on throttle reversal.
struct DrivelineCompliance {
    Real stiffness = 0.0; // k_s (N*m/rad)
    Real damping = 0.0;   // c_s (N*m*s/rad)
    Real backlash = 0.0;  // beta (rad)
};

struct DrivelineState {
    Real twist = 0.0;     // Delta theta (rad)
    Real twistRate = 0.0; // Delta omega (rad/s)
};

// Torque through the shaft (N*m), T = k_s Delta theta_eff + c_s Delta omega.
Real computeDrivelineTorque(const DrivelineCompliance& compliance, const DrivelineState& state);

} // namespace sim::vehicle
