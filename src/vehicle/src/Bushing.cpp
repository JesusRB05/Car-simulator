// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Bushing.hpp"

namespace sim::vehicle {

BushingForces computeBushingForces(const BushingCoefficients& coefficients, const BushingState& state) {
    BushingForces forces;
    // Diagonal, so the force and moment are component-wise and restore the
    // displacement (negative sign).
    forces.force = -coefficients.translationalStiffness * state.displacement;
    forces.moment = -coefficients.rotationalStiffness * state.rotation;
    return forces;
}

} // namespace sim::vehicle
