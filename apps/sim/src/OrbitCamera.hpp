// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <glm/glm.hpp>

namespace sim::render {
class Camera;
class Mesh;
class Scene;
} // namespace sim::render

namespace sim::app {

// Mouse-driven orbit camera for the inspection scenes. The eye sits on a
// sphere around `target`: yaw is measured around +Y from +Z towards +X and
// pitch upwards from the horizon, so yaw = pitch = 0 looks down -Z (the
// engine's forward) from a point on +Z. Right-handed, +Y up.
struct OrbitCamera
{
    glm::vec3 target{0.0f};
    float yaw = 0.0f;
    float pitch = 0.3f;
    float distance = 10.0f;
    float fovDegrees = 60.0f;

    // Eye position for the current parameters.
    glm::vec3 eye() const;

    // Adopts the view of an existing camera. The orbit target is placed on the
    // camera's gaze, at the depth of `focus` along it.
    void fromCamera(const render::Camera& camera, const glm::vec3& focus);

    // Keeps the direction and moves target/distance so a sphere of the given
    // centre and radius fills the vertical field of view.
    void frame(const glm::vec3& center, float radius);

    // Mouse deltas in pixels.
    void orbit(float dx, float dy);
    void pan(float dx, float dy, float viewportHeight);
    void zoom(float wheelSteps);

    // Writes the view and projection into `camera`.
    void apply(render::Camera& camera, float aspect) const;
};

// Bounding sphere of every drawable of `scene` except those using `exclude`
// (typically the ground plane). Returns false when nothing is left.
bool sceneBounds(const render::Scene& scene, const render::Mesh* exclude, glm::vec3& center, float& radius);

} // namespace sim::app
