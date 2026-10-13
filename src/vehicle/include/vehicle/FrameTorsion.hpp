// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Kart frame torsion (PHYSICS_PLAN Section 5.6). A kart has no springs or dampers;
// its compliance is the chassis twisting about the longitudinal axis between the
// front and rear halves, M = k_t (theta_f - theta_r) + c_t (dtheta_f - dtheta_r).
struct FrameTorsion {
    Real stiffness = 0.0; // k_t (N*m/rad)
    Real damping = 0.0;   // c_t (N*m*s/rad)
};

Real computeFrameTorsionMoment(const FrameTorsion& torsion, Real frontAngle, Real rearAngle, Real frontRate,
                               Real rearRate);

} // namespace sim::vehicle
