// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Anti-roll bar (PHYSICS_PLAN Section 5.3). A bar of torsional stiffness k_theta
// at lateral arm spacing t_a ties the two wheels of an axle: the difference in
// their vertical travel twists it and it answers with an equal and opposite
// force pair, so it adds roll stiffness without adding ride stiffness.
//
// The forces are vertical, in the same sense as the spring force of each corner
// (positive pushes the wheel down). The bar's own motion ratio is folded into
// the arm spacing.
struct AntiRollBarForces {
    Real left = 0.0;  // N
    Real right = 0.0; // N
};

AntiRollBarForces computeAntiRollBarForces(Real torsionalStiffness, Real armSpacing, Real leftTravel, Real rightTravel);

} // namespace sim::vehicle
