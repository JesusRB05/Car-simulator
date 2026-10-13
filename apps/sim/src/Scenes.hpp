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

// Scene categories:
//  - "render_tests": static scenes that exercise the renderer (geometry,
//    depth, lighting, orientation); captured in screenshots/render_tests_sheet.png.
//  - "physics": real PhysicsWorld demos (see PhysicsScenes.hpp); captured in
//    screenshots/contact_sheet.png.
const std::vector<std::string>& renderTestSceneNames();

// The names of every built-in demo scene, in display order (render tests first,
// then physics scenes).
std::vector<std::string> sceneNames();

// Browser grouping of the built-in scenes.
enum class SceneCategory
{
    Physics,    // live PhysicsWorld scenes stepped with the fixed step
    RenderTest, // static scenes used by the headless render checks
};

// One entry of the scene registry shown by the interactive scene browser.
struct SceneInfo
{
    std::string name;
    SceneCategory category = SceneCategory::RenderTest;
    std::string description;
};

// Every scene the interactive app can load (physics scenes first, then the
// render tests, including harness-only scenes such as "orientation"), with a
// one-line description each.
const std::vector<SceneInfo>& sceneCatalog();

// Human-readable label of a category ("Physics", "Render tests").
const char* sceneCategoryLabel(SceneCategory category);

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
