// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "telemetry/Telemetry.hpp"

#include <numeric>
#include <sstream>

#include "vfs/VirtualFileSystem.hpp"

namespace sim::telemetry {

Telemetry::Telemetry(std::size_t capacity) : capacity_(capacity < 1 ? 1 : capacity) {}

void Telemetry::beginFrame(double nowSeconds) {
    frameStart_ = nowSeconds;
    pending_ = FrameSample{};
}

void Telemetry::recordPhysics(double milliseconds, int steps) {
    pending_.physicsTimeMs += milliseconds;
    pending_.physicsSteps += steps;
}

void Telemetry::recordRender(double milliseconds) {
    pending_.renderTimeMs += milliseconds;
}

void Telemetry::recordUi(double milliseconds) {
    pending_.uiTimeMs += milliseconds;
}

void Telemetry::recordGpu(double milliseconds) {
    if (milliseconds >= 0.0) {
        pending_.gpuTimeMs = milliseconds;
    }
}

void Telemetry::setSimulationTime(double seconds) {
    pending_.simulationTime = seconds;
}

void Telemetry::endFrame(double nowSeconds) {
    pending_.frameTimeMs = (nowSeconds - frameStart_) * 1000.0;
    last_ = pending_;
    last_.fps = last_.frameTimeMs > 0.0 ? 1000.0 / last_.frameTimeMs : 0.0;

    history_.push_back(last_);
    while (history_.size() > capacity_) {
        history_.pop_front();
    }
}

double Telemetry::averageFrameTimeMs() const {
    if (history_.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (const FrameSample& sample : history_) {
        total += sample.frameTimeMs;
    }
    return total / static_cast<double>(history_.size());
}

double Telemetry::averageFps() const {
    const double average = averageFrameTimeMs();
    return average > 0.0 ? 1000.0 / average : 0.0;
}

bool Telemetry::exportCsv(vfs::VirtualFileSystem& vfs, std::string_view virtualPath) const {
    std::ostringstream stream;
    stream << "frame_time_ms,physics_ms,render_ms,ui_ms,gpu_ms,simulation_time,physics_steps,fps\n";
    for (const FrameSample& sample : history_) {
        stream << sample.frameTimeMs << ',' << sample.physicsTimeMs << ',' << sample.renderTimeMs << ','
               << sample.uiTimeMs << ',' << sample.gpuTimeMs << ',' << sample.simulationTime << ','
               << sample.physicsSteps << ',' << sample.fps << '\n';
    }
    return vfs.writeText(virtualPath, stream.str());
}

} // namespace sim::telemetry
