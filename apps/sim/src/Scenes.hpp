// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <vector>

namespace sim::render {
class Scene;
class Mesh;
} // namespace sim::render

namespace sim::app {

// A point that must be visible at a known screen position with a roughly known
// colour. Used by the headless semantic checks to prove that the camera, the
// depth test and the interpolation actually placed geometry where expected.
struct SceneProbe
{
    glm::vec3 world;
    glm::vec4 color;
};

// The names of every built-in demo scene, in display order.
std::vector<std::string> sceneNames();

// Applies the shared demo key light (direction + colour) to the given scene.
// Both the static render-test scenes and the physics scenes use it.
void setDefaultLight(sim::render::Scene& scene);

// Rebuilds `scene` (using the shared `cube` and `ground` meshes) as the demo
// scene named `name`. Unknown names fall back to "default" and return false.
bool buildScene(const std::string& name, sim::render::Scene& scene, const std::shared_ptr<sim::render::Mesh>& cube,
                const std::shared_ptr<sim::render::Mesh>& ground);

// Probe points for the given scene (empty if the scene is unknown).
std::vector<SceneProbe> sceneProbes(const std::string& name);

} // namespace sim::app
