// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "core/Clock.hpp"

#include <algorithm>
#include <chrono>

#include "core/Assert.hpp"

namespace sim::core {

FixedStepClock::FixedStepClock(double fixedDt, std::uint32_t maxStepsPerFrame)
    : fixedDt_(fixedDt), maxStepsPerFrame_(maxStepsPerFrame) {
    SIM_ASSERT(fixedDt > 0.0, "fixed step must be positive");
}

void FixedStepClock::setFixedDt(double dt) {
    SIM_ASSERT(dt > 0.0, "fixed step must be positive");
    fixedDt_ = dt;
}

StepResult FixedStepClock::advance(double frameDelta) {
    StepResult result;

    if (frameDelta < 0.0) {
        frameDelta = 0.0;
    }

    accumulator_ += frameDelta;

    const auto available = static_cast<std::uint64_t>(accumulator_ / fixedDt_);
    const auto steps = std::min<std::uint64_t>(available, maxStepsPerFrame_);

    accumulator_ -= static_cast<double>(steps) * fixedDt_;
    simTime_ += static_cast<double>(steps) * fixedDt_;

    // Guard against unbounded accumulation if the frame rate is chronically
    // below the simulation rate.
    if (available > steps) {
        accumulator_ = std::min(accumulator_, fixedDt_ * static_cast<double>(maxStepsPerFrame_));
    }

    result.steps = static_cast<std::uint32_t>(steps);
    result.simTime = simTime_;
    result.alpha = std::clamp(accumulator_ / fixedDt_, 0.0, 1.0);
    return result;
}

void FixedStepClock::reset(double simTime) {
    accumulator_ = 0.0;
    simTime_ = simTime;
}

FrameClock::FrameClock() : lastTime_(now()) {}

double FrameClock::now() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double FrameClock::tick() {
    const double current = now();
    lastDelta_ = current - lastTime_;
    lastTime_ = current;
    totalTime_ += lastDelta_;
    ++frameCount_;
    return lastDelta_;
}

} // namespace sim::core
