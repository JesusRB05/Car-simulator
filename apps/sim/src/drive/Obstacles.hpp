// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Static scenery for the driving scene, used to exercise the rigid-body
// collision path (the raycast car's chassis is a box that now collides with the
// world). The layout is generated procedurally from the track so it is
// deterministic, and every obstacle doubles as a render description (shape,
// pose, colour, label) so the scene can draw it without a second list.

#include <cstdint>
#include <string>
#include <vector>

#include "drive/Track.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"

namespace sim::app::drive {

// One static obstacle. `shape` selects which dimensions matter: a Box uses
// `halfExtents`, a Cylinder uses `radius` and `halfHeight` (local +Y axis).
struct Obstacle {
    physics::ShapeType shape = physics::ShapeType::Box;
    Vec3 position{0.0};
    physics::Quat orientation{1.0, 0.0, 0.0, 0.0};
    Vec3 halfExtents{0.5, 0.5, 0.5}; // Box: local half-sizes
    Real radius = 0.5;               // Cylinder
    Real halfHeight = 0.5;           // Cylinder
    std::uint32_t rgba = 0xFFFFFFFF; // straight (non-premultiplied) RGBA colour
    std::string kind;                // "barrier", "block", "tyre", "cone", "ramp"
};

// The full procedural layout: barrier walls lining both sides of the circuit,
// plus a run-off collision test yard beside the start straight (concrete blocks,
// tyre stacks, a cone slalom and a ramp). Deterministic for a given track.
std::vector<Obstacle> buildObstacleLayout(const Track& track);

// Appends every obstacle to `world` as a static, collision-enabled body, in the
// same order. Static bodies carry no mass and are never integrated.
void addObstaclesToWorld(physics::PhysicsWorld& world, const std::vector<Obstacle>& obstacles);

} // namespace sim::app::drive
