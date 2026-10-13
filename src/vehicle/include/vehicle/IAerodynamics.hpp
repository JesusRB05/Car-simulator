// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;
using physics::Vec3;

// Aerodynamic loads on the chassis, in the chassis frame (+X right, +Y up,
// -Z forward), referred to the chassis reference point.
struct AeroLoads
{
    Vec3 force{0.0};  // N; drag points to +Z when driving forward, downforce is -Y
    Vec3 moment{0.0}; // N*m about the reference point
};

// Aerodynamic model of the vehicle body: drag, lift/downforce and the
// resulting pitch/yaw/roll moments.
class IAerodynamics
{
public:
    virtual ~IAerodynamics() = default;

    // Computes loads from the air velocity relative to the chassis, in the
    // chassis frame (m/s; a car driving forward in still air sees air moving
    // towards +Z), and the air density (kg/m^3).
    virtual AeroLoads computeLoads(const Vec3& airVelocityChassis, Real airDensity) const = 0;
};

} // namespace sim::vehicle
