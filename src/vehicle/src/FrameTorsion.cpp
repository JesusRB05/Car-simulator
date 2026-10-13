// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/FrameTorsion.hpp"

namespace sim::vehicle {

Real computeFrameTorsionMoment(const FrameTorsion& torsion, Real frontAngle, Real rearAngle, Real frontRate,
                               Real rearRate) {
    return torsion.stiffness * (frontAngle - rearAngle) + torsion.damping * (frontRate - rearRate);
}

} // namespace sim::vehicle
