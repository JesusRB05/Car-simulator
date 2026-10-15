// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "input/ForceFeedback.hpp"

#include <algorithm>
#include <cmath>

namespace sim::input {

namespace {

double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

} // namespace

void ForceFeedbackController::startTest(double seconds) {
    testRemaining_ = std::max(0.0, seconds);
    testPhase_ = 0.0;
    stopRequested_ = false;
}

FfbCommand ForceFeedbackController::update(const FfbInput& input, bool devicePresent, bool windowFocused, double dt) {
    FfbCommand command;

    const bool allowed = settings_.enabled && devicePresent && windowFocused && !stopRequested_ && settings_.caps.any();
    if (!allowed) {
        active_ = false;
        impactEnvelope_ = 0.0;
        testRemaining_ = 0.0;
        return command; // active = false: caller stops every running effect
    }

    active_ = true;
    const double step = std::max(0.0, dt);

    // Impact is a short, decaying envelope on top of the steady effects.
    const double impact = clamp(input.impact, 0.0, 1.0);
    impactEnvelope_ = std::max(impactEnvelope_ * std::exp(-step * 12.0), impact);

    // Understeer lightens the self-aligning torque; oversteer adds a kick in the
    // direction the car is already turning.
    double baseTorque = clamp(input.steeringTorque, -1.0, 1.0);
    baseTorque *= 1.0 - 0.6 * clamp(input.understeer, 0.0, 1.0);
    baseTorque += 0.5 * clamp(input.oversteer, 0.0, 1.0) * std::copysign(1.0, baseTorque != 0.0 ? baseTorque : 1.0);

    double constant = baseTorque * settings_.constantForceGain;
    if (settings_.invertDirection) {
        constant = -constant;
    }
    constant = clamp(constant, -settings_.maxForce, settings_.maxForce);
    constant = clamp(constant * settings_.masterGain, -1.0, 1.0);

    // Condition effects scale with speed so the wheel is light at a standstill.
    const double speedGate = clamp(input.speed, 0.0, 1.0);
    const double spring = clamp(settings_.springGain * speedGate * settings_.masterGain, 0.0, 1.0);
    const double damper = clamp(settings_.damperGain * speedGate * settings_.masterGain, 0.0, 1.0);
    const double friction = clamp(settings_.frictionGain * speedGate * settings_.masterGain, 0.0, 1.0);

    // Rumble carries the kerb / road texture and a little of the impact.
    const double texture = clamp(input.kerbRumble, 0.0, 1.0);
    const double rumbleAmount =
        clamp((texture + 0.7 * impactEnvelope_) * settings_.rumbleGain * settings_.masterGain, 0.0, 1.0);

    // Self-test: a gentle oscillation, independent of the car.
    if (testRemaining_ > 0.0) {
        testPhase_ += step;
        testRemaining_ = std::max(0.0, testRemaining_ - step);
        const double oscillation = std::sin(testPhase_ * 6.283185307179586 * 0.5);
        constant = 0.25 * oscillation * settings_.masterGain;
        if (settings_.invertDirection) {
            constant = -constant;
        }
    }

    command.active = true;
    command.constantForce = settings_.caps.constantForce ? constant : 0.0;
    command.spring = settings_.caps.spring ? spring : 0.0;
    command.damper = settings_.caps.damper ? damper : 0.0;
    command.friction = settings_.caps.friction ? friction : 0.0;
    command.rumbleLow = settings_.caps.rumble ? rumbleAmount : 0.0;
    command.rumbleHigh = settings_.caps.rumble ? rumbleAmount : 0.0;
    return command;
}

} // namespace sim::input
