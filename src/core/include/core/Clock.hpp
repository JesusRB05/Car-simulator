// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>

namespace sim::core {

// Result of advancing a fixed-step simulation clock for one rendered frame.
struct StepResult
{
    std::uint32_t steps = 0;  // number of fixed steps to simulate this frame
    double alpha = 0.0;       // interpolation factor in [0, 1) between the last two states
    double simTime = 0.0;     // simulated time after the steps have been applied
};

// Fixed-step clock. The render loop feeds it the real frame time; it returns
// how many fixed simulation steps to run, plus the interpolation alpha used to
// blend the two most recent simulation states for the renderer.
//
// The clock clamps the amount of catch-up work to avoid the "spiral of death"
// after a long stall (window drag, breakpoint, ...).
class FixedStepClock
{
public:
    explicit FixedStepClock(double fixedDt = 1.0 / 1000.0, std::uint32_t maxStepsPerFrame = 8);

    StepResult advance(double frameDelta);

    double fixedDt() const { return fixedDt_; }
    void setFixedDt(double dt);
    double simTime() const { return simTime_; }
    std::uint32_t maxStepsPerFrame() const { return maxStepsPerFrame_; }

    // Discards accumulated time and (optionally) resets the simulated clock.
    void reset(double simTime = 0.0);

private:
    double fixedDt_;
    double accumulator_ = 0.0;
    double simTime_ = 0.0;
    std::uint32_t maxStepsPerFrame_;
};

// Wall-clock helper for frame timing, based on std::chrono::steady_clock.
class FrameClock
{
public:
    FrameClock();

    // Returns the seconds elapsed since the previous call (first call returns 0).
    double tick();

    double lastDelta() const { return lastDelta_; }
    double totalTime() const { return totalTime_; }
    std::uint64_t frameCount() const { return frameCount_; }

private:
    double lastTime_;
    double lastDelta_ = 0.0;
    double totalTime_ = 0.0;
    std::uint64_t frameCount_ = 0;

    static double now();
};

} // namespace sim::core
