// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace sim::physics {
class PhysicsWorld;
} // namespace sim::physics

namespace sim::render {
class DebugDraw;
class Mesh;
class Scene;
} // namespace sim::render

namespace sim::app {

// The names of the built-in physics demo scenes, in display order. Every scene
// is a real PhysicsWorld (bodies, joints and force elements) stepped with the
// deterministic fixed step; the static render-test scenes live in sceneNames().
const std::vector<std::string>& physicsSceneNames();

// One-line description of a physics scene (empty for unknown names).
const char* physicsSceneDescription(const std::string& name);

// True when `name` names one of the physics scenes.
bool isPhysicsScene(const std::string& name);

// Builds the initial state of the given physics scene into `world` and returns
// the simulated seconds that should elapse before the scene is representative
// (headless captures keep stepping deterministically for that long).
double buildPhysicsWorld(const std::string& name, sim::physics::PhysicsWorld& world);

// Frames the camera for a physics scene so the settled motion is in shot.
void physicsSceneCamera(const std::string& name, sim::render::Scene& scene, float aspect);

// Maps the current state of `world` into `scene` (shared `ground` plane plus one
// mesh per body, sized by its physics shape; static bodies grey, dynamic bodies
// hue-coded so the joints read at a glance).
void buildSceneFromWorld(const sim::physics::PhysicsWorld& world, sim::render::Scene& scene,
                         const std::shared_ptr<sim::render::Mesh>& cube, const std::shared_ptr<sim::render::Mesh>& sphere,
                         const std::shared_ptr<sim::render::Mesh>& cylinder, const std::shared_ptr<sim::render::Mesh>& ground);

// Total mechanical energy of a physics scene: kinetic energy plus gravitational
// potential of every dynamic body, plus the elastic energy of the scene's
// springs (whose parameters the scene builders own).
double physicsSceneEnergy(const std::string& name, const sim::physics::PhysicsWorld& world);

// Layers of the physics debug overlay, toggled from the interactive UI.
struct PhysicsDebugOptions
{
    bool reference = true;  // analytic reference curves (projectile parabola)
    bool joints = true;     // rods, arms and constraint links
    bool springs = true;    // spring coils
    bool bodyAxes = true;   // body-frame axes
    bool velocities = true; // linear velocity vectors of dynamic bodies
    // Generic per-body layers on top of the scene-specific ones (axes of every
    // dynamic body, links of every chain joint). Off for the headless captures
    // so their images stay stable.
    bool genericOverlays = true;
};

// Scene-specific debug overlay lines (analytic projectiles, joints/rods, spring
// coils, body and spin axes) driven by the current world state. This overload
// is the fixed overlay used by the headless captures.
void drawPhysicsSceneDebug(const std::string& name, const sim::physics::PhysicsWorld& world, sim::render::DebugDraw& draw);

// Same overlay with individually selectable layers.
void drawPhysicsSceneDebug(const std::string& name, const sim::physics::PhysicsWorld& world, sim::render::DebugDraw& draw,
                           const PhysicsDebugOptions& options);

} // namespace sim::app