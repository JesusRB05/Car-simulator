// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "OrbitCamera.hpp"
#include "render/Scene.hpp"

namespace sim::rhi {
class Device;
} // namespace sim::rhi

namespace sim::app {

struct CarSpec;

// The showroom for the selected car. There is no glTF pipeline for the cars yet
// (content/base/cars/*/body.gltf is an empty placeholder), so the preview is
// assembled from the engine primitives exactly like the in-game car, driven by
// the real VehicleSetup, on a dark floor with an orbit camera. It is real 3D:
// the player rotates the car with the mouse.
class GarageScene {
public:
    explicit GarageScene(sim::rhi::Device& device);

    // Rebuilds the model for `car` and frames the camera on it.
    void setCar(const CarSpec& car);

    // Re-frames the camera on the current car (used when the screen reopens).
    void resetView();

    OrbitCamera& camera() { return camera_; }

    // Fills `scene` with the floor and the car and applies the camera.
    void buildScene(sim::render::Scene& scene, float aspect);

private:
    void rebuild();
    void frameCamera();

    sim::rhi::Device& device_;
    std::shared_ptr<sim::render::Mesh> cube_;
    std::shared_ptr<sim::render::Mesh> cylinder_;
    std::shared_ptr<sim::render::Mesh> floor_;

    std::vector<sim::render::Drawable> carParts_;
    glm::vec3 carCenter_{0.0f, 0.6f, 0.0f};
    float carRadius_ = 3.2f;

    OrbitCamera camera_;
};

} // namespace sim::app
