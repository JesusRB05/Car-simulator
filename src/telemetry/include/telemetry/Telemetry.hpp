// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <deque>
#include <string_view>

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::telemetry {

struct FrameSample {
    double frameTimeMs = 0.0;
    double physicsTimeMs = 0.0;
    double renderTimeMs = 0.0;
    double uiTimeMs = 0.0;
    double gpuTimeMs = 0.0;
    double simulationTime = 0.0;
    int physicsSteps = 0;
    double fps = 0.0;
};

// A rolling record of frame timings. The application fills one sample per
// rendered frame; ImGui panels and the CSV exporter read from it.
class Telemetry {
public:
    explicit Telemetry(std::size_t capacity = 600);

    void beginFrame(double nowSeconds);
    void recordPhysics(double milliseconds, int steps);
    void recordRender(double milliseconds);
    void recordUi(double milliseconds);
    void recordGpu(double milliseconds);
    void setSimulationTime(double seconds);
    void endFrame(double nowSeconds);

    double frameTimeMs() const { return last_.frameTimeMs; }
    double physicsTimeMs() const { return last_.physicsTimeMs; }
    double renderTimeMs() const { return last_.renderTimeMs; }
    double uiTimeMs() const { return last_.uiTimeMs; }
    double gpuTimeMs() const { return last_.gpuTimeMs; }
    double fps() const { return last_.fps; }
    int physicsSteps() const { return last_.physicsSteps; }

    double averageFrameTimeMs() const;
    double averageFps() const;

    const std::deque<FrameSample>& history() const { return history_; }
    const FrameSample& last() const { return last_; }

    // Appends one CSV line per sample through the VFS. Returns false on failure.
    bool exportCsv(vfs::VirtualFileSystem& vfs, std::string_view virtualPath) const;

private:
    std::size_t capacity_;
    std::deque<FrameSample> history_;
    FrameSample last_;
    FrameSample pending_;
    double frameStart_ = 0.0;
};

} // namespace sim::telemetry
