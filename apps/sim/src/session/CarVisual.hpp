// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// GPU half of the car model: the uploaded meshes plus the rig that says what
// each one moves with. The loader is shared by the drive scene and the garage;
// both then place the same parts, one from the physics, one at rest.

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/Model.hpp"
#include "session/CarRig.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::rhi {
class Device;
}

namespace sim::render {
class Scene;
}

namespace sim::app {

struct CarSkin;

// One uploaded primitive, ready to draw.
struct CarVisualPart {
    std::shared_ptr<render::Mesh> mesh;
    render::Material material;
    CarRigKind kind = CarRigKind::Chassis;
    int corner = -1;
    glm::mat4 local{1.0f};
};

struct CarVisual {
    std::vector<CarVisualPart> parts;
    std::array<CarRigWheel, 4> wheels{};
    // Model space, from the glTF accessors.
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};

    bool valid() const { return !parts.empty(); }
    bool hasWheels() const;
};

struct CarVisualOptions {
    // Skin maps to substitute while the model uploads.
    std::vector<render::MaterialOverride> overrides;
    // A skin that ships no paint atlas may still colour the car: the flat
    // colour is applied to `paintMaterial` (and only when that material ends up
    // without a base-colour map).
    std::string paintMaterial = "paint_livery";
    bool hasBodyColor = false;
    glm::vec4 bodyColor{1.0f};
};

// Loads and uploads `modelPath`. Returns nullptr when the file is missing or
// malformed; `warnings` collects the rig and skin issues.
std::shared_ptr<CarVisual> loadCarVisual(rhi::Device& device, const vfs::VirtualFileSystem& vfs,
                                         std::string_view modelPath, const CarVisualOptions& options = {},
                                         std::vector<std::string>* warnings = nullptr);

// The upload options for a car wearing `skin` (nullptr keeps the model's own
// materials). Shared by the drive scene and the showroom so both render the
// same car.
CarVisualOptions carVisualOptions(const CarSkin* skin);

// Adds every part of the car to `scene`.
//
// `chassis` is the body frame with the model origin on the ground (the glTF
// convention puts the car's ground plane at y = 0 in the root's space).
// `hubs` are the world frames of the four wheel pivots before the spin, and
// `spin` is the accumulated rolling angle per corner, in radians.
void addCarVisual(render::Scene& scene, const CarVisual& visual, const glm::mat4& chassis,
                  const std::array<glm::mat4, 4>& hubs, const std::array<float, 4>& spin);

// Hub frames at rest from the model's own pivot positions, for a showroom that
// has no physics (`chassis` is the body frame, usually the identity or a lift).
std::array<glm::mat4, 4> restHubFrames(const CarVisual& visual, const glm::mat4& chassis);

} // namespace sim::app
