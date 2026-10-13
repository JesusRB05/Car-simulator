// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Engine.hpp"

#include <cmath>
#include <cstddef>

namespace sim::vehicle {

namespace {

// Locates x within an increasing grid and returns the bracketing indices and the
// fraction between them. Values off the ends are clamped (constant extrapolation).
void locate(const std::vector<Real>& grid, Real x, std::size_t& i0, std::size_t& i1, Real& t) {
    const std::size_t n = grid.size();
    if (n == 1) {
        i0 = 0;
        i1 = 0;
        t = 0.0;
        return;
    }
    if (x <= grid.front()) {
        i0 = 0;
        i1 = 1;
        t = 0.0;
        return;
    }
    if (x >= grid.back()) {
        i0 = n - 2;
        i1 = n - 1;
        t = 1.0;
        return;
    }
    i0 = 0;
    while (i0 + 1 < n && x > grid[i0 + 1]) {
        ++i0;
    }
    i1 = i0 + 1;
    t = (x - grid[i0]) / (grid[i1] - grid[i0]);
}

} // namespace

bool EngineTorqueMap::empty() const {
    return speed.empty() || throttle.empty() || torque.size() != speed.size() * throttle.size();
}

Real EngineTorqueMap::torqueAt(Real angularVelocity, Real throttlePosition) const {
    if (empty()) {
        return 0.0;
    }
    std::size_t i0 = 0;
    std::size_t i1 = 0;
    std::size_t j0 = 0;
    std::size_t j1 = 0;
    Real ts = 0.0;
    Real tt = 0.0;
    locate(speed, angularVelocity, i0, i1, ts);
    locate(throttle, throttlePosition, j0, j1, tt);

    const std::size_t nt = throttle.size();
    const Real v00 = torque[i0 * nt + j0];
    const Real v01 = torque[i0 * nt + j1];
    const Real v10 = torque[i1 * nt + j0];
    const Real v11 = torque[i1 * nt + j1];

    const Real bottom = v00 + tt * (v01 - v00);
    const Real top = v10 + tt * (v11 - v10);
    return bottom + ts * (top - bottom);
}

Real EngineFriction::torqueAt(Real angularVelocity) const {
    return constant + linear * angularVelocity + quadratic * angularVelocity * angularVelocity;
}

Real Turbocharger::targetBoost(Real engineAngularVelocity, Real throttle) const {
    Real speedFactor = engineAngularVelocity / boostReferenceSpeed;
    if (speedFactor < 0.0) {
        speedFactor = 0.0;
    } else if (speedFactor > 1.0) {
        speedFactor = 1.0;
    }
    const Real clampedThrottle = (throttle < 0.0) ? 0.0 : (throttle > 1.0 ? 1.0 : throttle);
    Real target = ambientPressure + (maxBoostPressure - ambientPressure) * clampedThrottle * speedFactor;
    if (antiLag && clampedThrottle <= antiLagThreshold && target < antiLagBoostPressure) {
        target = antiLagBoostPressure;
    }
    return target;
}

Real Turbocharger::boostTorque(Real naturallyAspiratedTorque, Real boostPressure) const {
    return naturallyAspiratedTorque * (1.0 + boostGain * (boostPressure / ambientPressure - 1.0));
}

Real stepTurboBoost(const Turbocharger& turbo, Real boostPressure, Real targetBoost, Real deltaSeconds) {
    if (turbo.timeConstant <= 0.0) {
        return targetBoost;
    }
    const Real decay = std::exp(-deltaSeconds / turbo.timeConstant);
    return targetBoost + (boostPressure - targetBoost) * decay;
}

void Engine::reset() {
    angularVelocity_ = 0.0;
    effectiveThrottle_ = 0.0;
    idleIntegral_ = 0.0;
    fuelCut_ = false;
}

Real Engine::step(Real deltaSeconds, Real throttle, Real loadTorque) {
    if (config.throttleTimeConstant > 0.0) {
        const Real decay = std::exp(-deltaSeconds / config.throttleTimeConstant);
        effectiveThrottle_ = throttle + (effectiveThrottle_ - throttle) * decay;
    } else {
        effectiveThrottle_ = throttle;
    }

    if (angularVelocity_ > config.revLimit) {
        fuelCut_ = true;
    } else if (angularVelocity_ < config.revLimit - config.revLimitHysteresis) {
        fuelCut_ = false;
    }

    const Real command = fuelCut_ ? 0.0 : effectiveThrottle_;
    Real torque = map.torqueAt(angularVelocity_, command);

    if (effectiveThrottle_ <= config.idleThrottleThreshold) {
        const Real error = config.idleSpeed - angularVelocity_;
        if (error > 0.0) {
            idleIntegral_ += error * deltaSeconds;
            const Real cap = 50.0; // anti-windup on the accumulated error
            if (idleIntegral_ > cap) {
                idleIntegral_ = cap;
            }
            torque += config.idleProportionalGain * error + config.idleIntegralGain * idleIntegral_;
        } else {
            idleIntegral_ = 0.0;
        }
    } else {
        idleIntegral_ = 0.0;
    }

    const Real net = torque - friction.torqueAt(angularVelocity_) - loadTorque;
    angularVelocity_ += net / config.inertia * deltaSeconds;
    if (angularVelocity_ < 0.0) {
        angularVelocity_ = 0.0;
    }
    return torque;
}

} // namespace sim::vehicle
