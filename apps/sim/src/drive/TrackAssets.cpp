// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/TrackAssets.hpp"

#include <utility>

#include "session/TrackCatalog.hpp"

namespace sim::app::drive {

std::shared_ptr<DriveTrackAssets> loadDriveTrackAssets(rhi::Device* device, const vfs::VirtualFileSystem& vfs,
                                                       const TrackSpec& spec, std::vector<std::string>* warnings) {
    if (spec.modelPath.empty()) {
        return nullptr;
    }
    auto assets = std::make_shared<DriveTrackAssets>();
    auto model = std::make_shared<TrackModel>();
    std::string error;
    if (!loadTrackModel(vfs, spec.modelPath, TrackModelOptions{}, *model, error)) {
        if (warnings != nullptr) {
            warnings->push_back(error);
        }
        return nullptr;
    }
    if (warnings != nullptr) {
        for (const std::string& warning : model->warnings) {
            warnings->push_back(spec.modelPath + ": " + warning);
        }
    }
    if (device != nullptr) {
        assets->visual = uploadTrackVisual(*device, *model);
    }
    assets->model = std::move(model);
    if (spec.layout && spec.layout->hasStart) {
        assets->hasStart = true;
        assets->startPosition = physics::Vec3(spec.layout->start.x, spec.layout->start.y, spec.layout->start.z);
        assets->startHeading = spec.layout->startHeadingRadians;
    }
    return assets;
}

} // namespace sim::app::drive
