// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/WheelSpin.hpp"

#include <algorithm>
#include <cmath>

namespace sim::vehicle {
namespace {

Real slipRatioFor(Real spin, Real effectiveRadius, Real contactSpeed, Real denominator) {
    return (spin * effectiveRadius - contactSpeed) / denominator;
}

} // namespace

WheelSpinResult stepWheelSpin(const WheelSpinCoefficients& c, const ITireModel& tire, const TireState& context,
                              const WheelSpinInput& input, Real dt, WheelSpinState& state) {
    WheelSpinResult result;
    const Real re = input.effectiveRadius;
    const Real vcx = input.contactSpeed;
    const Real denominator = std::max(std::abs(vcx), c.minContactSpeed);

    const auto forceAt = [&](Real omega) {
        TireState s = context;
        s.contactSpeed = vcx;
        s.slipRatio = slipRatioFor(omega, re, vcx, denominator);
        return tire.computeForces(s).longitudinal;
    };

    const Real omega = state.spin;
    const Real fx = forceAt(omega);

    // Linearise the tyre force about the current spin with a central difference;
    // this is the k = dFx/dOmega of the plan.
    const Real step = (c.derivativeStep > 0.0) ? c.derivativeStep : 1.0e-4;
    const Real k = (forceAt(omega + step) - forceAt(omega - step)) / (2.0 * step);

    // Omega_{n+1} = (J Omega + dt (T - My - Re (Fx - k Omega))) / (J + dt Re k).
    const Real jw = c.inertia;
    const Real torque = input.driveTorque - input.brakeTorque;
    const Real numerator = jw * omega + dt * (torque - input.rollingResistanceMoment - re * (fx - k * omega));
    const Real implicitDenominator = jw + dt * re * k;
    Real nextSpin = (std::abs(implicitDenominator) > 1e-12) ? numerator / implicitDenominator : omega;

    // A braked wheel must not reverse through zero: it locks as static friction.
    bool locked = false;
    if (input.brakeTorque > 0.0 && nextSpin < 0.0) {
        nextSpin = 0.0;
        locked = true;
    }

    state.spin = nextSpin;
    state.locked = locked;

    result.spin = nextSpin;
    result.slipRatio = slipRatioFor(nextSpin, re, vcx, denominator);
    result.longitudinalForce = forceAt(nextSpin);
    result.locked = locked;
    return result;
}

} // namespace sim::vehicle
