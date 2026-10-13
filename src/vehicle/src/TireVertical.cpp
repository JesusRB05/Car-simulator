// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/TireVertical.hpp"

#include <algorithm>
#include <cmath>

namespace sim::vehicle {

TireVerticalResult evaluateTireVertical(const TireVerticalCoefficients& c, const TireVerticalState& s) {
    TireVerticalResult result;
    const Real delta = std::max(0.0, s.deflection);

    // One-sided spring-damper: no tension, so an airborne corner (delta = 0) with
    // a receding contact carries no load.
    const Real stiffness = c.kz0 + c.kz1 * delta;
    result.normalLoad = std::max(0.0, stiffness * delta + c.cz * s.deflectionRate);

    result.loadedRadius = c.unloadedRadius - delta;

    const Real fz0 = c.nominalLoad;
    const Real loadRatio = (fz0 != 0.0) ? result.normalLoad / fz0 : 0.0;
    const Real deflectionScale = (c.kz0 != 0.0) ? fz0 / c.kz0 : 0.0;
    result.effectiveRadius = c.unloadedRadius -
                             deflectionScale * (c.radiusD * std::atan(c.radiusB * loadRatio) + c.radiusF * loadRatio) +
                             c.radiusOmega * s.spin * s.spin * c.unloadedRadius;

    const Real r0 = c.unloadedRadius;
    const Real rl = std::min(result.loadedRadius, r0);
    result.contactHalfLength = (rl < r0) ? std::sqrt(r0 * r0 - rl * rl) : 0.0;

    return result;
}

} // namespace sim::vehicle
