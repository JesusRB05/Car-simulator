// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Mat3;
using physics::Real;
using physics::Vec3;

// Mass properties of the vehicle (PHYSICS_PLAN Section 6.1). The sprung body,
// each unsprung mass, the fuel, the driver and any ballast are separate
// contributions; the total inertia about the combined centre of mass follows from
// the parallel-axis theorem, so it is recomputed whenever a mass moves or burns.
//
// position and inertia are in the vehicle frame (+X right, +Y up, -Z forward).
struct MassContribution {
    Real mass = 0.0;              // kg
    Vec3 position{0.0, 0.0, 0.0}; // m
    Mat3 inertia{0.0};            // kg*m^2, about its own CG, about the vehicle axes
};

struct MassProperties {
    Real mass = 0.0;        // kg
    Vec3 centreOfMass{0.0}; // m
    Mat3 inertia{0.0};      // kg*m^2, about the combined centre of mass
};

MassProperties combineMassProperties(const std::vector<MassContribution>& contributions);

} // namespace sim::vehicle
