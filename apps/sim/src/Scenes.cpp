// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "Scenes.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <array>
#include <cmath>

#include "PhysicsScenes.hpp"
#include "render/Mesh.hpp"
#include "render/Scene.hpp"

namespace sim::app {
namespace {

constexpr float kAspect = 16.0f / 9.0f;

// Direction the light travels; the shader negates it to get the surface-to-light
// vector, so pointing it downward lights the top faces of the boxes.
const glm::vec3 kLightDirection = glm::normalize(glm::vec3(-0.45f, -1.0f, -0.40f));
const glm::vec3 kLightColor{1.0f, 0.97f, 0.92f};

const glm::vec4 kGround{0.30f, 0.33f, 0.28f, 1.0f};

void addBox(render::Scene& scene, const std::shared_ptr<render::Mesh>& cube, const glm::vec3& center,
            const glm::vec3& size, const glm::vec4& color, const glm::mat4& rotation = glm::mat4(1.0f)) {
    const glm::mat4 model = glm::translate(glm::mat4(1.0f), center) * rotation * glm::scale(glm::mat4(1.0f), size);
    scene.add(cube, model, color);
}

void setCamera(render::Scene& scene, const glm::vec3& eye, const glm::vec3& target) {
    scene.camera().setPerspective(60.0f, kAspect, 0.1f, 1000.0f);
    scene.camera().lookAt(eye, target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::vec4 hueColor(float t) {
    const float h = t - std::floor(t);
    const float x = 1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f);
    if (h < 1.0f / 6.0f) {
        return {1.0f, x, 0.0f, 1.0f};
    }
    if (h < 2.0f / 6.0f) {
        return {x, 1.0f, 0.0f, 1.0f};
    }
    if (h < 3.0f / 6.0f) {
        return {0.0f, 1.0f, x, 1.0f};
    }
    if (h < 4.0f / 6.0f) {
        return {0.0f, x, 1.0f, 1.0f};
    }
    if (h < 5.0f / 6.0f) {
        return {x, 0.0f, 1.0f, 1.0f};
    }
    return {1.0f, 0.0f, x, 1.0f};
}

} // namespace

const std::vector<std::string>& renderTestSceneNames() {
    static const std::vector<std::string> names{"default", "grid",    "pyramid", "ramp",       "orbit",
                                                "tower",   "checker", "rows",    "orientation"};
    return names;
}

std::vector<std::string> sceneNames() {
    const std::vector<std::string>& renderTests = renderTestSceneNames();
    const std::vector<std::string>& physicsScenes = physicsSceneNames();
    std::vector<std::string> names;
    names.reserve(renderTests.size() + physicsScenes.size());
    names.insert(names.end(), renderTests.begin(), renderTests.end());
    names.insert(names.end(), physicsScenes.begin(), physicsScenes.end());
    return names;
}

const std::vector<SceneInfo>& sceneCatalog() {
    static const std::vector<SceneInfo> catalog = [] {
        std::vector<SceneInfo> entries;
        entries.push_back({"drive", SceneCategory::Driving,
                           "Keyboard-drivable raycast car on a test circuit (WASD, Space, R, C, G)."});
        for (const std::string& name : physicsSceneNames()) {
            entries.push_back({name, SceneCategory::Physics, physicsSceneDescription(name)});
        }
        const std::vector<SceneInfo> renderTests{
            {"default", SceneCategory::RenderTest, "Single car-sized box: the baseline lighting and depth check."},
            {"grid", SceneCategory::RenderTest, "6x6 grid of alternating boxes seen from above."},
            {"pyramid", SceneCategory::RenderTest, "Four stacked layers of boxes with hue-coded levels."},
            {"ramp", SceneCategory::RenderTest, "Row of boxes of increasing height."},
            {"orbit", SceneCategory::RenderTest, "Ring of rotated boxes around the origin."},
            {"tower", SceneCategory::RenderTest, "Eight-box vertical tower."},
            {"checker", SceneCategory::RenderTest, "Thin checkerboard tiles just above the ground (depth precision)."},
            {"rows", SceneCategory::RenderTest, "Three coloured rows of boxes."},
            {"orientation", SceneCategory::RenderTest, "Off-axis boxes that prove +Y is up and +X is right on screen."},
        };
        entries.insert(entries.end(), renderTests.begin(), renderTests.end());
        return entries;
    }();
    return catalog;
}

const char* sceneCategoryLabel(SceneCategory category) {
    switch (category) {
    case SceneCategory::Driving:
        return "Driving";
    case SceneCategory::Physics:
        return "Physics";
    case SceneCategory::RenderTest:
        break;
    }
    return "Render tests";
}

void setDefaultLight(render::Scene& scene) {
    scene.setDirectionalLight(kLightDirection, kLightColor);
}

bool buildScene(const std::string& name, render::Scene& scene, const std::shared_ptr<render::Mesh>& cube,
                const std::shared_ptr<render::Mesh>& ground) {
    scene.clear();
    setDefaultLight(scene);
    scene.add(ground, glm::mat4(1.0f), kGround);

    if (name == "grid") {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                const float x = (static_cast<float>(i) - 2.5f) * 2.4f;
                const float z = (static_cast<float>(j) - 2.5f) * 2.4f;
                const glm::vec4 color =
                    ((i + j) % 2 == 0) ? glm::vec4(0.85f, 0.30f, 0.20f, 1.0f) : glm::vec4(0.20f, 0.45f, 0.85f, 1.0f);
                addBox(scene, cube, {x, 0.70f, z}, {1.40f, 1.40f, 1.40f}, color);
            }
        }
        // Look at the centre of the (blue) box at i=5, j=4 so the semantic probe
        // reliably lands on that box.
        setCamera(scene, {16.0f, 17.0f, 16.0f}, {6.0f, 0.70f, 3.6f});
        return true;
    }

    if (name == "pyramid") {
        const int layers = 4;
        for (int level = 0; level < layers; ++level) {
            const int count = layers - level;
            const float spacing = 1.7f;
            const float y = static_cast<float>(level) * 1.5f + 0.75f;
            const glm::vec4 color = hueColor(0.55f + 0.08f * static_cast<float>(level));
            for (int i = 0; i < count; ++i) {
                for (int j = 0; j < count; ++j) {
                    const float x = (static_cast<float>(i) - (count - 1) * 0.5f) * spacing;
                    const float z = (static_cast<float>(j) - (count - 1) * 0.5f) * spacing;
                    addBox(scene, cube, {x, y, z}, {1.5f, 1.5f, 1.5f}, color);
                }
            }
        }
        setCamera(scene, {9.0f, 8.0f, 11.0f}, {0.0f, 5.25f, 0.0f});
        return true;
    }

    if (name == "ramp") {
        for (int i = 0; i < 6; ++i) {
            const float x = (static_cast<float>(i) - 2.5f) * 1.6f;
            const float height = 0.5f + static_cast<float>(i) * 0.6f;
            addBox(scene, cube, {x, height * 0.5f, 0.0f}, {1.5f, height, 1.5f},
                   hueColor(0.45f + 0.07f * static_cast<float>(i)));
        }
        setCamera(scene, {10.0f, 6.0f, 12.0f}, {0.8f, 1.15f, 0.0f});
        return true;
    }

    if (name == "orbit") {
        const int count = 8;
        for (int i = 0; i < count; ++i) {
            const float angle = static_cast<float>(i) / static_cast<float>(count) * 6.2831853f;
            const glm::vec3 center(6.0f * std::sin(angle), 0.6f, 6.0f * std::cos(angle));
            const glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 1.0f, 0.0f));
            addBox(scene, cube, center, {1.2f, 1.2f, 1.2f}, hueColor(static_cast<float>(i) / static_cast<float>(count)),
                   rotation);
        }
        setCamera(scene, {0.0f, 12.0f, 16.0f}, {0.0f, 0.6f, 6.0f});
        return true;
    }

    if (name == "tower") {
        for (int i = 0; i < 8; ++i) {
            const float y = (static_cast<float>(i) + 0.5f) * 1.4f;
            const glm::vec4 color =
                (i % 2 == 0) ? glm::vec4(0.90f, 0.75f, 0.20f, 1.0f) : glm::vec4(0.75f, 0.20f, 0.45f, 1.0f);
            addBox(scene, cube, {0.0f, y, 0.0f}, {1.6f, 1.4f, 1.6f}, color);
        }
        setCamera(scene, {8.0f, 7.0f, 11.0f}, {0.0f, 4.9f, 0.0f});
        return true;
    }

    if (name == "checker") {
        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j < 8; ++j) {
                const float x = (static_cast<float>(i) - 3.5f) * 1.8f;
                const float z = (static_cast<float>(j) - 3.5f) * 1.8f;
                const glm::vec4 color =
                    ((i + j) % 2 == 0) ? glm::vec4(0.85f, 0.30f, 0.20f, 1.0f) : glm::vec4(0.15f, 0.20f, 0.75f, 1.0f);
                // Tiles are thin slabs floating a couple of millimetres above
                // the ground. With reversed-Z the huge ground quad can win a
                // grazing depth tie against geometry resting right on it, so a
                // small-but-material separation is needed: at ~18 m with
                // near=0.1, far=1000 a D32Sfloat buffer resolves ~0.2 mm, so a
                // 2 mm gap is ~10 depth units and never ties. The thickness is
                // 10 mm so the top face is at y = 0.012 (where the probe sits).
                addBox(scene, cube, {x, 0.007f, z}, {1.8f, 0.010f, 1.8f}, color);
            }
        }
        setCamera(scene, {0.0f, 13.0f, 14.0f}, {0.9f, 0.012f, 0.9f});
        return true;
    }

    if (name == "rows") {
        const std::array<glm::vec4, 3> colors{glm::vec4(0.85f, 0.25f, 0.25f, 1.0f),
                                              glm::vec4(0.25f, 0.75f, 0.35f, 1.0f),
                                              glm::vec4(0.30f, 0.40f, 0.90f, 1.0f)};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 5; ++col) {
                const float x = (static_cast<float>(col) - 2.0f) * 2.0f;
                const float z = (static_cast<float>(row) - 1.0f) * 2.4f;
                addBox(scene, cube, {x, 0.65f, z}, {1.3f, 1.3f, 1.3f}, colors[static_cast<std::size_t>(row)]);
            }
        }
        setCamera(scene, {13.0f, 7.0f, 15.0f}, {0.0f, 0.65f, 0.0f});
        return true;
    }

    // "orientation": test harness for the absolute screen orientation. A
    // horizontal camera gazes down -Z while four coloured boxes are placed off
    // the gaze axis: one clearly higher, one on each side of the reference.
    // With the RHI viewport flip in place, +Y world must be the top of the
    // image and +X world the right, so each box lands where its probe expects;
    // a vertically flipped image fails the probes. The "sky top / ground
    // bottom" horizon is checked separately by the headless runner
    // (checkOrientation), without using the projection matrix.
    if (name == "orientation") {
        addBox(scene, cube, {0.0f, 0.9f, -8.0f}, {1.0f, 1.0f, 1.0f}, glm::vec4(0.85f, 0.35f, 0.30f, 1.0f)); // reference
        addBox(scene, cube, {0.0f, 2.7f, -8.0f}, {1.0f, 1.0f, 1.0f},
               glm::vec4(0.30f, 0.75f, 0.90f, 1.0f)); // high (above)
        addBox(scene, cube, {-4.5f, 0.5f, -8.0f}, {1.0f, 1.0f, 1.0f}, glm::vec4(0.30f, 0.70f, 0.40f, 1.0f)); // left
        addBox(scene, cube, {4.5f, 0.5f, -8.0f}, {1.0f, 1.0f, 1.0f}, glm::vec4(0.75f, 0.30f, 0.70f, 1.0f));  // right
        setCamera(scene, {0.0f, 0.5f, 2.0f}, {0.0f, 0.5f, -8.0f});
        return true;
    }

    // "default" and any unknown name.
    addBox(scene, cube, {0.0f, 0.8f, 0.0f}, {1.9f, 1.6f, 4.2f}, glm::vec4(0.80f, 0.15f, 0.12f, 1.0f));
    setCamera(scene, {6.0f, 4.0f, 12.0f}, {0.0f, 0.8f, 0.0f});
    return name == "default";
}

std::vector<SceneProbe> sceneProbes(const std::string& name) {
    // Every probe sits at the centre of the box the camera now looks at, so it
    // projects to the screen centre and lands on that box regardless of aspect.
    // Physics scenes move, so they carry no probes: their deterministically
    // stepped golden images (plus the CPU physics tests) pin their state.
    if (isPhysicsScene(name)) {
        return {};
    }
    if (name == "grid") {
        return {{glm::vec3(6.0f, 0.70f, 3.6f), glm::vec4(0.20f, 0.45f, 0.85f, 1.0f)}};
    }
    if (name == "pyramid") {
        return {{glm::vec3(0.0f, 5.25f, 0.0f), hueColor(0.55f + 0.08f * 3.0f)}};
    }
    if (name == "ramp") {
        return {{glm::vec3(0.8f, 1.15f, 0.0f), hueColor(0.45f + 0.07f * 3.0f)}};
    }
    if (name == "orbit") {
        return {{glm::vec3(0.0f, 0.6f, 6.0f), hueColor(0.0f)}};
    }
    if (name == "tower") {
        return {{glm::vec3(0.0f, 4.9f, 0.0f), glm::vec4(0.75f, 0.20f, 0.45f, 1.0f)}};
    }
    if (name == "checker") {
        return {{glm::vec3(0.9f, 0.012f, 0.9f), glm::vec4(0.85f, 0.30f, 0.20f, 1.0f)}};
    }
    if (name == "rows") {
        return {{glm::vec3(0.0f, 0.65f, 0.0f), glm::vec4(0.25f, 0.75f, 0.35f, 1.0f)}};
    }
    // The orientation harness: probes sit on off-axis boxes so a horizontally
    // or vertically flipped image cannot be confused for an upright one.
    if (name == "orientation") {
        return {{glm::vec3(0.0f, 0.9f, -8.0f), glm::vec4(0.85f, 0.35f, 0.30f, 1.0f)},
                {glm::vec3(0.0f, 2.7f, -8.0f), glm::vec4(0.30f, 0.75f, 0.90f, 1.0f)},
                {glm::vec3(-4.5f, 0.5f, -8.0f), glm::vec4(0.30f, 0.70f, 0.40f, 1.0f)},
                {glm::vec3(4.5f, 0.5f, -8.0f), glm::vec4(0.75f, 0.30f, 0.70f, 1.0f)}};
    }
    return {{glm::vec3(0.0f, 0.8f, 0.0f), glm::vec4(0.80f, 0.15f, 0.12f, 1.0f)}};
}

} // namespace sim::app
