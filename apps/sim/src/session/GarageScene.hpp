// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "OrbitCamera.hpp"
#include "render/Scene.hpp"

namespace sim::rhi {
class Device;
} // namespace sim::rhi

namespace sim::vfs {
class VirtualFileSystem;
} // namespace sim::vfs

namespace sim::app {

struct CarSpec;
struct CarSkin;
struct CarVisual;

// The showroom for the selected car. When the car's content ships a glTF model
// it is uploaded and shown with the selected skin; a car without one falls back
// to the engine primitives driven by the real VehicleSetup. Either way the
// preview is real 3D: the player rotates the car with the mouse.
class GarageScene {
public:
    GarageScene(sim::rhi::Device& device, const sim::vfs::VirtualFileSystem& vfs);

    // Rebuilds the model for `car`, wearing `skin` (may be null). The upload is
    // cached: switching skins only reloads when the model or the skin changes.
    void setCar(const CarSpec& car, const CarSkin* skin = nullptr);

    // Re-frames the camera on the current car (used when the screen reopens).
    void resetView();

    OrbitCamera& camera() { return camera_; }

    // Fills `scene` with the floor and the car and applies the camera.
    void buildScene(sim::render::Scene& scene, float aspect);

private:
    void rebuild();
    void frameCamera();
    void buildPrimitives(const CarSpec& car);

    sim::rhi::Device& device_;
    const sim::vfs::VirtualFileSystem& vfs_;
    std::shared_ptr<sim::render::Mesh> cube_;
    std::shared_ptr<sim::render::Mesh> cylinder_;
    std::shared_ptr<sim::render::Mesh> floor_;

    // The uploaded model and which (car, skin) pair produced it.
    std::shared_ptr<CarVisual> visual_;
    std::string carRef_;
    std::string skinRef_;

    std::vector<sim::render::Drawable> carParts_;
    glm::vec3 carCenter_{0.0f, 0.6f, 0.0f};
    float carRadius_ = 3.2f;

    OrbitCamera camera_;
};

} // namespace sim::app
