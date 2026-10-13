// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/SpringDamper.hpp"

#include <algorithm>
#include <cmath>

namespace sim::vehicle {

Real DamperTable::forceAt(Real speed) const {
    if (speeds.empty()) {
        return 0.0;
    }
    if (speed <= speeds.front()) {
        return forces.front();
    }
    if (speed >= speeds.back()) {
        return forces.back();
    }
    const auto upper = std::upper_bound(speeds.begin(), speeds.end(), speed);
    const std::size_t i = static_cast<std::size_t>(upper - speeds.begin());
    const Real s0 = speeds[i - 1];
    const Real s1 = speeds[i];
    const Real f0 = forces[i - 1];
    const Real f1 = forces[i];
    const Real span = s1 - s0;
    if (span <= 0.0) {
        return f0;
    }
    const Real t = (speed - s0) / span;
    return f0 + t * (f1 - f0);
}

SpringDamperForces computeSpringDamperForces(const SpringDamperCoefficients& c, const SpringDamperState& s) {
    SpringDamperForces f;

    f.spring = c.stiffness * (c.restLength - s.length) + c.preload;

    if (!c.damperTable.empty()) {
        f.damper = c.damperTable.forceAt(s.velocity);
    } else if (s.velocity >= 0.0) {
        f.damper = c.dampingCompression * s.velocity;
    } else {
        f.damper = c.dampingRebound * s.velocity;
    }

    if (c.hasBumpStop && s.length < c.bumpStopLength) {
        const Real x = c.bumpStopLength - s.length;
        f.bumpStop = c.bumpStopStiffness * std::pow(x, c.bumpStopExponent) + c.bumpStopDamping * s.velocity;
    }

    if (c.hasReboundStop && s.length > c.reboundStopLength) {
        const Real x = s.length - c.reboundStopLength;
        f.reboundStop =
            -(c.reboundStopStiffness * std::pow(x, c.reboundStopExponent) + c.reboundStopDamping * s.velocity);
    }

    return f;
}

} // namespace sim::vehicle
