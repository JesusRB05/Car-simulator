// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// The circuit content a descriptor points at, imported once and shared by the
// drive scene. Collision is always built; the GPU visual only when a device is
// present, so the headless path can drive the same circuit without a renderer.

#include <memory>
#include <string>
#include <vector>

#include "drive/TrackModel.hpp"
#include "drive/TrackVisual.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::rhi {
class Device;
}

namespace sim::app {
struct TrackSpec;
}

namespace sim::app::drive {

struct DriveTrackAssets {
    std::shared_ptr<const TrackModel> model;
    std::shared_ptr<const TrackVisual> visual;
    bool hasStart = false;
    physics::Vec3 startPosition{0.0}; // only x and z are used: the mesh sets y
    double startHeading = 0.0;
};

// Imports the circuit `spec` names. Returns nullptr when the descriptor has no
// model (a purely procedural circuit) or when the model cannot be read; in the
// latter case the failure is appended to `warnings` and the caller falls back to
// the analytic circuit. `device` may be null, in which case only collision is
// built.
std::shared_ptr<DriveTrackAssets> loadDriveTrackAssets(rhi::Device* device, const vfs::VirtualFileSystem& vfs,
                                                       const TrackSpec& spec,
                                                       std::vector<std::string>* warnings = nullptr);

} // namespace sim::app::drive
