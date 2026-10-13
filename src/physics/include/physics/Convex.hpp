// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Generic convex-convex intersection (Phase 1, P1.2): a GJK boolean/penetration
// test and an EPA expansion, driven entirely by each shape's support function.
// This fills the combinations the analytic narrowphase does not cover (convex
// hulls, and any future primitive) and gives a single-point contact with a
// penetration depth and witness points on both bodies.

#include "physics/Math.hpp"
#include "physics/PhysicsTypes.hpp"
#include "physics/RigidBody.hpp"

namespace sim::physics {

// The farthest world-space point of `body` along `direction`, for any convex
// shape supported here (Box, Sphere, Cylinder, Convex).
Vec3 supportWorld(const RigidBody& body, const Vec3& direction);

// Winding-agnostic GJK separation test. True when the two convex shapes overlap
// (including exact touching).
bool convexOverlap(const RigidBody& a, const RigidBody& b);

struct ConvexContact {
    bool intersecting = false;
    Vec3 normal{0.0, 1.0, 0.0}; // unit, pointing from A to B
    Real depth = 0.0;           // non-negative penetration along `normal`
    Vec3 pointA{0.0};           // witness on A (world)
    Vec3 pointB{0.0};           // witness on B (world)
};

// Full convex-convex contact. When the shapes are separated the result has
// `intersecting == false` and `depth == 0`.
ConvexContact convexContact(const RigidBody& a, const RigidBody& b);

} // namespace sim::physics
