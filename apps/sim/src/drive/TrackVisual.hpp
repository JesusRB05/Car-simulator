// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// GPU half of the circuit model: the uploaded tiles and their materials. Kept
// apart from the device-free import (`TrackModel`) for the same reason the car
// splits `CarRig` from `CarVisual`: the import rules are testable without a
// device, and the headless path can build the collision without uploading.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/Scene.hpp"

namespace sim::rhi {
class Device;
}

namespace sim::app::drive {

struct TrackModel;

struct TrackVisual {
    std::vector<std::shared_ptr<render::Mesh>> meshes;
    std::vector<render::Drawable> drawables; // transform is always the identity
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};
    std::size_t triangleCount = 0;

    bool valid() const { return !drawables.empty(); }
};

// Uploads every tile of `model` and its materials, sharing one texture per
// image. Returns nullptr when there is nothing to draw.
std::shared_ptr<TrackVisual> uploadTrackVisual(rhi::Device& device, const TrackModel& model);

// Adds the circuit to `scene`.
void addTrackVisual(render::Scene& scene, const TrackVisual& visual);

} // namespace sim::app::drive
