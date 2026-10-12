// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/PhysicsWorld.hpp"

#include <cstring>

#include "physics/Integrator.hpp"

namespace sim::physics {
namespace {

void hashMix(std::uint64_t& hash, double value)
{
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(double));
    hash ^= bits;
    hash *= 1099511628211ULL;
}

} // namespace

RigidBody& PhysicsWorld::createBody()
{
    bodies_.emplace_back();
    RigidBody& created = bodies_.back();
    created.updateCache();
    return created;
}

void PhysicsWorld::step(Real dt)
{
    for (RigidBody& body : bodies_)
    {
        body.updateCache();
        body.clearAccumulators();
    }

    for (auto& element : forceElements_)
    {
        element->apply();
    }

    for (RigidBody& body : bodies_)
    {
        integrator::integrateVelocities(body, gravity, dt);
    }

    for (auto& constraint : constraints_)
    {
        constraint->prepare(dt);
    }
    for (auto& constraint : constraints_)
    {
        constraint->warmStart();
    }
    for (int iteration = 0; iteration < velocityIterations; ++iteration)
    {
        for (auto& constraint : constraints_)
        {
            constraint->solveVelocity();
        }
    }

    for (RigidBody& body : bodies_)
    {
        integrator::integratePositions(body, dt);
    }

    time_ += static_cast<double>(dt);
}

Real PhysicsWorld::advance(Real elapsed)
{
    accumulator_ += elapsed;
    while (accumulator_ >= fixedStep)
    {
        step(fixedStep);
        accumulator_ -= fixedStep;
    }
    return accumulator_;
}

std::uint64_t PhysicsWorld::stateHash() const
{
    std::uint64_t hash = 1469598103934665603ULL;
    for (const RigidBody& body : bodies_)
    {
        const double values[13] = {body.position.x,   body.position.y,        body.position.z,
                                   body.orientation.w, body.orientation.x,    body.orientation.y,
                                   body.orientation.z, body.linearVelocity.x, body.linearVelocity.y,
                                   body.linearVelocity.z, body.angularVelocity.x, body.angularVelocity.y,
                                   body.angularVelocity.z};
        for (double value : values)
        {
            hashMix(hash, value);
        }
    }
    hashMix(hash, time_);
    return hash;
}

WorldSnapshot PhysicsWorld::saveSnapshot() const
{
    WorldSnapshot snapshot;
    snapshot.time = time_;
    snapshot.bodies.reserve(bodies_.size());
    for (const RigidBody& body : bodies_)
    {
        BodyState state;
        state.position = body.position;
        state.orientation = body.orientation;
        state.linearVelocity = body.linearVelocity;
        state.angularVelocity = body.angularVelocity;
        snapshot.bodies.push_back(state);
    }
    snapshot.impulses.reserve(constraints_.size());
    for (const auto& constraint : constraints_)
    {
        std::vector<Real> impulses;
        impulses.reserve(constraint->rows.size());
        for (const ConstraintRow& row : constraint->rows)
        {
            impulses.push_back(row.impulse);
        }
        snapshot.impulses.push_back(std::move(impulses));
    }
    return snapshot;
}

void PhysicsWorld::restoreSnapshot(const WorldSnapshot& snapshot)
{
    const std::size_t count = std::min(bodies_.size(), snapshot.bodies.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        RigidBody& body = bodies_[i];
        body.position = snapshot.bodies[i].position;
        body.orientation = snapshot.bodies[i].orientation;
        body.linearVelocity = snapshot.bodies[i].linearVelocity;
        body.angularVelocity = snapshot.bodies[i].angularVelocity;
        body.updateCache();
    }
    const std::size_t constraintCount = std::min(constraints_.size(), snapshot.impulses.size());
    for (std::size_t i = 0; i < constraintCount; ++i)
    {
        Constraint& constraint = *constraints_[i];
        const std::vector<Real>& impulses = snapshot.impulses[i];
        const std::size_t rowCount = std::min(constraint.rows.size(), impulses.size());
        for (std::size_t j = 0; j < rowCount; ++j)
        {
            constraint.rows[j].impulse = impulses[j];
        }
    }
    time_ = snapshot.time;
    accumulator_ = 0.0;
}

} // namespace sim::physics
