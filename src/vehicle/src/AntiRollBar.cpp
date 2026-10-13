// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/AntiRollBar.hpp"

namespace sim::vehicle {

AntiRollBarForces computeAntiRollBarForces(Real torsionalStiffness, Real armSpacing, Real leftTravel,
                                           Real rightTravel) {
    AntiRollBarForces forces;
    if (armSpacing <= 0.0) {
        return forces;
    }
    // Compress more on one side and the bar pushes that side back up while
    // pushing the other side down: an equal and opposite couple.
    const Real stiffness = torsionalStiffness / (armSpacing * armSpacing);
    const Real couple = stiffness * (leftTravel - rightTravel);
    forces.left = -couple;
    forces.right = couple;
    return forces;
}

} // namespace sim::vehicle
