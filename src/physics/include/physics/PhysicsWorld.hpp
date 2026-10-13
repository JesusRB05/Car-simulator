// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include "physics/Constraint.hpp"
#include "physics/ForceElements.hpp"
#include "physics/Joints.hpp"
#include "physics/RigidBody.hpp"
#include "physics/PhysicsTypes.hpp"

namespace sim::physics {

using BodyId = std::uint32_t;
inline constexpr BodyId kInvalidBody = 0;

// Rigid-body state used for the save/restore snapshots that the test harness and
// the interactive reset button rely on.
struct BodyState
{
    Vec3 position{0.0};
    Quat orientation{1.0, 0.0, 0.0, 0.0};
    Vec3 linearVelocity{0.0};
    Vec3 angularVelocity{0.0};
};

struct WorldSnapshot
{
    double time = 0.0;
    std::vector<BodyState> bodies;
    std::vector<std::vector<Real>> impulses;
};

// Owns bodies, joints and force elements and advances them with a deterministic
// fixed step. Bodies are stored in a deque so pointers handed out by
// createBody() stay valid for the lifetime of the world.
class PhysicsWorld
{
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

    RigidBody& createBody();
    std::size_t bodyCount() const { return bodies_.size(); }
    RigidBody& body(std::size_t index) { return bodies_[index]; }
    const RigidBody& body(std::size_t index) const { return bodies_[index]; }

    template <typename T, typename... Args>
    T& createConstraint(RigidBody& bodyA, RigidBody& bodyB, Args&&... args)
    {
        auto constraint = std::make_unique<T>(std::forward<Args>(args)...);
        constraint->bodyA = &bodyA;
        constraint->bodyB = &bodyB;
        constraints_.push_back(std::move(constraint));
        return static_cast<T&>(*constraints_.back());
    }

    std::size_t constraintCount() const { return constraints_.size(); }

    template <typename T, typename... Args>
    T& createForceElement(RigidBody* bodyA, RigidBody* bodyB, Args&&... args)
    {
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
    std::vector<std::vector<Constraint*>> buildIslands() const;
    void solveVelocities(const std::vector<std::vector<Constraint*>>& islands);
    void projectPositions(const std::vector<std::vector<Constraint*>>& islands, Real dt);

    std::deque<RigidBody> bodies_;
    std::deque<std::unique_ptr<Constraint>> constraints_;
    std::deque<std::unique_ptr<ForceElement>> forceElements_;
    Real accumulator_ = 0.0;
    double time_ = 0.0;
};

} // namespace sim::physics
