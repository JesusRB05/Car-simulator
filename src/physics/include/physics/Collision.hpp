// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Shape-vs-shape collision detection for the physics engine: world-space AABBs,
// a deterministic AABB-tree broadphase and analytic narrowphase routines.
//
// This is the detection half of Phase 1 (see docs/ROADMAP.md). It finds contacts
// but does not yet resolve them: the solver integration that turns these
// manifolds into impulse rows lands next. Everything here is double precision,
// pure (no hidden state) and ordered, so its output is reproducible.

#include <cstddef>
#include <vector>

#include "physics/Math.hpp"
#include "physics/PhysicsTypes.hpp"
#include "physics/RigidBody.hpp"

namespace sim::physics {

class PhysicsWorld;

// World-space axis-aligned bounding box.
struct Aabb {
    Vec3 min{0.0};
    Vec3 max{0.0};

    bool overlaps(const Aabb& other) const {
        return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y && max.y >= other.min.y &&
               min.z <= other.max.z && max.z >= other.min.z;
    }

    Vec3 center() const { return 0.5 * (min + max); }
    Vec3 extent() const { return 0.5 * (max - min); }
};

// World-space AABB of a body's shape at its current pose (velocities ignored).
Aabb computeAabb(const RigidBody& body);

// A single contact of a manifold. `normal` is a unit vector pointing from body A
// towards body B. `penetration` is signed: a positive value is the overlap depth
// along the normal, a negative value is a speculative contact whose magnitude is
// the still-open gap when the shapes are within the search margin (see `collide`).
struct ContactPoint {
    Vec3 position{0.0};
    Vec3 normal{0.0};
    Real penetration = 0.0;
};

inline constexpr int kMaxContactPoints = 4;

// A contact between two bodies, addressed by their index in the world so the
// result stays valid across container moves.
struct ContactManifold {
    std::size_t bodyA = 0;
    std::size_t bodyB = 0;
    int count = 0;
    ContactPoint points[kMaxContactPoints];
};

// A candidate pair produced by the broadphase, ordered with bodyA < bodyB.
struct BodyPair {
    std::size_t bodyA = 0;
    std::size_t bodyB = 0;
};

// Deterministic broadphase: every collidable pair whose world-space AABBs
// overlap, ordered by (bodyA, bodyB). Backed by an AABB tree (physics/Broadphase.hpp);
// the signature hides the acceleration structure so scenes are unaware of it.
// When `velocityMarginDt > 0` each body's AABB is inflated by its own per-step
// travel (linear plus angular speed times the shape bounding radius), so a fast
// body is still paired with geometry it has not reached yet but could in one
// step; this is what lets the narrowphase build speculative contacts.
std::vector<BodyPair> broadphase(const PhysicsWorld& world, Real velocityMarginDt = 0.0);

// Narrowphase for a single pair. Writes up to `maxPoints` contacts and returns
// how many were produced (0 when the shapes are separated by more than `margin`
// or the combination is not supported yet). Normals point from `a` to `b`.
//
// `margin` enables speculative contacts: when the shapes are separated but their
// gap is at most `margin`, a contact is still emitted with a negative
// `penetration` equal to `-gap`. The solver turns that into a one-step "do not
// close more than the gap" constraint, which is what stops fast bodies from
// tunnelling through thin geometry. The default of 0 reports only real overlaps.
int collide(const RigidBody& a, const RigidBody& b, ContactPoint* out, int maxPoints, Real margin = 0.0);

// Broadphase + narrowphase over the whole world, in deterministic order. When
// `margin > 0` the manifolds may include speculative (negative-penetration)
// contacts. When `velocityMarginDt > 0` each pair's margin is further grown by
// that pair's relative speed times this interval, so fast bodies look far
// enough ahead to never tunnel through thin geometry (see PhysicsWorld::
// enableSpeculativeContacts).
std::vector<ContactManifold> detectCollisions(const PhysicsWorld& world, Real margin = 0.0,
                                              Real velocityMarginDt = 0.0);

} // namespace sim::physics
