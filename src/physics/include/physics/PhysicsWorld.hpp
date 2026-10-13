// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

#include "physics/Collision.hpp"
#include "physics/Constraint.hpp"
#include "physics/ForceElements.hpp"
#include "physics/Joints.hpp"
#include "physics/PhysicsTypes.hpp"
#include "physics/RigidBody.hpp"

namespace sim::physics {

using BodyId = std::uint32_t;
inline constexpr BodyId kInvalidBody = 0;

// Rigid-body state used for the save/restore snapshots that the test harness and
// the interactive reset button rely on.
struct BodyState {
    Vec3 position{0.0};
    Quat orientation{1.0, 0.0, 0.0, 0.0};
    Vec3 linearVelocity{0.0};
    Vec3 angularVelocity{0.0};
    bool sleeping = false;
    Real sleepTimer = 0.0;
};

// Warm-start impulses of one contact pair, captured so a restore resumes the
// contact solver exactly where it left off (joints use `impulses`).
struct ContactImpulseState {
    std::uint64_t key = 0;
    int count = 0;
    Real normal[kMaxContactPoints]{};
    Real tangent1[kMaxContactPoints]{};
    Real tangent2[kMaxContactPoints]{};
};

struct WorldSnapshot {
    double time = 0.0;
    std::vector<BodyState> bodies;
    std::vector<std::vector<Real>> impulses;
    std::vector<ContactImpulseState> contacts;
};

// Owns bodies, joints and force elements and advances them with a deterministic
// fixed step. Bodies are stored in a deque so pointers handed out by
// createBody() stay valid for the lifetime of the world.
class PhysicsWorld {
public:
    Vec3 gravity{0.0, -9.80665, 0.0};
    int velocityIterations = 8;
    Real fixedStep = 1.0 / 1000.0;

    // Constraint islands (joints connected through dynamic bodies) whose
    // bilateral rows fit in this count are solved with a dense LDL^T block
    // solve instead of plain sequential impulses: the equality rows are solved
    // exactly every iteration and only bounded rows (motors, limits) are
    // iterated. This is what keeps large mass ratios (1:1000) and long chains
    // rigid. 0 disables it and falls back to PGS for every island.
    int directSolverMaxRows = 128;

    // Position projection for rigid rows (SoftParams::rigid): after the
    // positions are integrated, up to `positionIterations` Newton passes move
    // the bodies back onto the constraint manifold (without touching the
    // velocities, so no energy is injected). A pass stops early once every
    // rigid row error is below `positionTolerance`; a single pass never moves a
    // row by more than `maxPositionCorrection`.
    int positionIterations = 4;
    Real positionTolerance = 1e-10;   // m or rad
    Real maxPositionCorrection = 0.2; // m or rad

    // Collision-response tuning. Only bodies whose RigidBody::collisionEnabled
    // is set take part. Each contact point is solved as one one-sided normal row
    // plus two Coulomb friction rows; the Baumgarte term removes penetration at
    // the velocity level and `contactSlop` is the resting penetration left
    // uncorrected so the response does not jitter. Friction and restitution come
    // from the bodies' material (see RigidBody::friction / restitution).
    Real contactBaumgarte = 0.2;
    Real contactSlop = 1e-3;

    // Speculative contacts. When enabled, the narrowphase search margin of a
    // pair grows with the bodies' relative speed (linear plus angular times the
    // shape's bounding radius) so a fast body is tested against the geometry
    // ahead of it and cannot tunnel through a thin wall in a single step. The
    // margin is `speculativeMargin * relativeSpeed * dt`, so a resting scene
    // (speed ~0) is unaffected; 0 disables the feature and only real overlaps
    // are reported.
    bool enableSpeculativeContacts = true;
    Real speculativeMargin = 1.0;

    // Sleeping. A dynamic body whose linear speed stays below
    // `sleepLinearThreshold` and angular speed below `sleepAngularThreshold`
    // for `sleepTime` seconds is frozen (velocities zeroed, not integrated and
    // treated as static by the solver) until it is woken by a moving body it is
    // in contact with or jointed to, by an applied impulse, or explicitly.
    bool enableSleeping = true;
    Real sleepLinearThreshold = 0.01;  // m/s
    Real sleepAngularThreshold = 0.05; // rad/s
    Real sleepTime = 0.5;              // s below the thresholds to fall asleep

    RigidBody& createBody();
    std::size_t bodyCount() const { return bodies_.size(); }
    RigidBody& body(std::size_t index) { return bodies_[index]; }
    const RigidBody& body(std::size_t index) const { return bodies_[index]; }

    template<typename T, typename... Args>
    T& createConstraint(RigidBody& bodyA, RigidBody& bodyB, Args&&... args) {
        auto constraint = std::make_unique<T>(std::forward<Args>(args)...);
        constraint->bodyA = &bodyA;
        constraint->bodyB = &bodyB;
        constraints_.push_back(std::move(constraint));
        return static_cast<T&>(*constraints_.back());
    }

    std::size_t constraintCount() const { return constraints_.size(); }

    template<typename T, typename... Args>
    T& createForceElement(RigidBody* bodyA, RigidBody* bodyB, Args&&... args) {
        auto element = std::make_unique<T>(std::forward<Args>(args)...);
        element->bodyA = bodyA;
        element->bodyB = bodyB;
        forceElements_.push_back(std::move(element));
        return static_cast<T&>(*forceElements_.back());
    }

    // Advances the simulation by exactly one step of the given size. Determinism
    // requires a fixed dt across runs.
    void step(Real dt);

    // Runs as many fixedStep() substeps as fit into `elapsed`, returning the
    // unconsumed remainder. The app uses this to drive the engine from wall time.
    Real advance(Real elapsed);

    double time() const { return time_; }

    // Order-sensitive FNV-1a hash over the full dynamic state, for the
    // determinism tests.
    std::uint64_t stateHash() const;

    WorldSnapshot saveSnapshot() const;
    void restoreSnapshot(const WorldSnapshot& snapshot);

private:
    // Rebuilds the transient contact constraints from the current overlap and
    // seeds them with the previous step's impulses (warm starting).
    void buildContacts(Real dt);
    void storeContactImpulses();

    // Wakes every sleeping body that a moving awake body is touching (given the
    // freshly detected manifolds), jointed to or coupled to through a force
    // element, propagating along the chain. Called before the contacts are
    // configured so a woken body's rows use its real mass.
    void wakeBodies(const std::vector<ContactManifold>& manifolds);
    // Advances each dynamic body's sleep timer and puts the quiet ones to sleep.
    void updateSleeping(Real dt);

    std::vector<std::vector<Constraint*>> buildIslands(const std::vector<Constraint*>& all) const;
    void solveVelocities(const std::vector<std::vector<Constraint*>>& islands);
    void projectPositions(const std::vector<std::vector<Constraint*>>& islands, Real dt);

    // Accumulated impulses of one contact pair, kept across steps so the solver
    // warm starts instead of rebuilding the stack from zero. `normal` is the
    // pushing impulse; `tangent1`/`tangent2` are the two friction directions.
    struct ContactImpulseCache {
        int count = 0;
        Real normal[kMaxContactPoints]{};
        Real tangent1[kMaxContactPoints]{};
        Real tangent2[kMaxContactPoints]{};
    };

    std::deque<RigidBody> bodies_;
    std::deque<std::unique_ptr<Constraint>> constraints_;
    std::deque<std::unique_ptr<ForceElement>> forceElements_;
    std::vector<std::unique_ptr<Constraint>> contacts_;
    std::vector<std::uint64_t> contactKeys_;
    std::unordered_map<std::uint64_t, ContactImpulseCache> contactCache_;
    Real accumulator_ = 0.0;
    double time_ = 0.0;
};

} // namespace sim::physics
