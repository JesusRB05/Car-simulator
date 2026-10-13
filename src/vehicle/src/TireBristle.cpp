// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/TireBristle.hpp"

#include <algorithm>
#include <cmath>

namespace sim::vehicle {

TireBristleResult updateTireBristle(const TireBristleCoefficients& c, Real normalLoad, Real vLong, Real vLat, Real dt,
                                    TireBristleState& state) {
    TireBristleResult result;
    const Real fz = std::max(0.0, normalLoad);
    const Real speed = std::sqrt(vLong * vLong + vLat * vLat);

    if (fz <= 0.0) {
        state.longitudinal = 0.0;
        state.lateral = 0.0;
        return result; // airborne: no bristle force
    }

    // Static friction ceiling g(v): Fs at rest decaying to Fc as the slip speed
    // rises (the Stribeck effect). The bristle force never exceeds it by much.
    const Real fs = c.staticFriction * fz;
    const Real fc = c.kineticFriction * fz;
    const Real stribeck =
        (c.stictionVelocity != 0.0) ? std::exp(-(speed / c.stictionVelocity) * (speed / c.stictionVelocity)) : 0.0;
    const Real g = std::max(fc + (fs - fc) * stribeck, 1e-9);

    const auto blendWeight = [&]() {
        if (c.blendSpeed <= 0.0) {
            return 1.0;
        }
        const Real t = std::min(1.0, speed / c.blendSpeed);
        return 1.0 - t * t * (3.0 - 2.0 * t); // 1 - smoothstep(t)
    };

    if (dt <= 0.0) {
        result.longitudinal = -(c.sigma0 * state.longitudinal + c.sigma2 * vLong);
        result.lateral = -(c.sigma0 * state.lateral + c.sigma2 * vLat);
        result.blend = std::clamp(blendWeight(), 0.0, 1.0);
        return result;
    }

    // Implicit bristle update, unconditionally stable at the physics step:
    //   z_{n+1} = (z_n + dt v) / (1 + dt sigma0 |v| / g).
    const Real denom = 1.0 + dt * c.sigma0 * speed / g;
    const Real zLong = (state.longitudinal + dt * vLong) / denom;
    const Real zLat = (state.lateral + dt * vLat) / denom;

    const Real zDotLong = (zLong - state.longitudinal) / dt;
    const Real zDotLat = (zLat - state.lateral) / dt;
    state.longitudinal = zLong;
    state.lateral = zLat;

    // The friction force opposes the slip velocity.
    result.longitudinal = -(c.sigma0 * zLong + c.sigma1 * zDotLong + c.sigma2 * vLong);
    result.lateral = -(c.sigma0 * zLat + c.sigma1 * zDotLat + c.sigma2 * vLat);
    result.blend = std::clamp(blendWeight(), 0.0, 1.0);
    return result;
}

} // namespace sim::vehicle
