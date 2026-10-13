// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Constraint.hpp"

#include <algorithm>
#include <cmath>

namespace sim::physics {

Real constraintMass(const ConstraintRow& row)
{
    Real k = 0.0;
    const RigidBody* a = row.bodyA;
    const RigidBody* b = row.bodyB;
    if (a != nullptr)
    {
        k += a->inverseMass * glm::dot(row.linearA, row.linearA);
        k += glm::dot(row.angularA, a->inverseInertiaWorld() * row.angularA);
    }
    if (b != nullptr)
    {
        k += b->inverseMass * glm::dot(row.linearB, row.linearB);
        k += glm::dot(row.angularB, b->inverseInertiaWorld() * row.angularB);
    }
    return k;
}

void configureRow(ConstraintRow& row, Real invEffectiveMass, Real positionError, Real dt, const SoftParams& soft)
{
    row.bias = 0.0;
    row.softness = 0.0;
    row.positionError = positionError;
    row.project = false;
    row.effectiveMass = invEffectiveMass > 0.0 ? 1.0 / invEffectiveMass : 0.0;

    if (soft.isRigid())
    {
        // Hard velocity row; the position error is removed by the world's
        // projection pass instead of a velocity bias, so nothing is injected.
        row.project = invEffectiveMass > 0.0;
        return;
    }

    const bool hasSpring = soft.stiffness > 0.0 || soft.frequency > 0.0;
    if (!hasSpring || dt <= 0.0 || invEffectiveMass <= 0.0)
    {
        return;  // velocity-only row (motors)
    }

    const Real m = 1.0 / invEffectiveMass;
    Real k = 0.0;
    if (soft.stiffness > 0.0)
    {
        k = soft.stiffness;
    }
    else
    {
        const Real omega = kTwoPiPhysics * soft.frequency;
        k = m * omega * omega;
    }
    // Critical damping of the (m, k) oscillator scaled by the damping ratio.
    const Real d = 2.0 * soft.dampingRatio * std::sqrt(k * m);
    const Real denominator = d + dt * k;
    if (denominator > 0.0)
    {
        row.softness = 1.0 / (dt * denominator);
        row.bias = positionError * dt * k * row.softness;
    }
    const Real total = invEffectiveMass + row.softness;
    row.effectiveMass = total > 0.0 ? 1.0 / total : 0.0;
}

void applyRowImpulse(ConstraintRow& row, Real lambda)
{
    if (lambda == 0.0)
    {
        return;
    }
    if (row.bodyA != nullptr)
    {
        row.bodyA->linearVelocity += row.bodyA->inverseMass * row.linearA * lambda;
        row.bodyA->angularVelocity += row.bodyA->inverseInertiaWorld() * row.angularA * lambda;
    }
    if (row.bodyB != nullptr)
    {
        row.bodyB->linearVelocity += row.bodyB->inverseMass * row.linearB * lambda;
        row.bodyB->angularVelocity += row.bodyB->inverseInertiaWorld() * row.angularB * lambda;
    }
}

void solveRow(ConstraintRow& row)
{
    Real cdot = 0.0;
    if (row.bodyA != nullptr)
    {
        cdot += glm::dot(row.linearA, row.bodyA->linearVelocity);
        cdot += glm::dot(row.angularA, row.bodyA->angularVelocity);
    }
    if (row.bodyB != nullptr)
    {
        cdot += glm::dot(row.linearB, row.bodyB->linearVelocity);
        cdot += glm::dot(row.angularB, row.bodyB->angularVelocity);
    }

    const Real delta = -row.effectiveMass * (cdot - row.velocityTarget + row.bias + row.softness * row.impulse);
    const Real newImpulse = std::clamp(row.impulse + delta, row.lower, row.upper);
    const Real applied = newImpulse - row.impulse;
    row.impulse = newImpulse;
    applyRowImpulse(row, applied);
}

void Constraint::rebuildRows(std::size_t count)
{
    std::vector<Real> saved(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i)
    {
        saved[i] = rows[i].impulse;
    }
    rows.assign(count, ConstraintRow{});
    for (std::size_t i = 0; i < count; ++i)
    {
        rows[i].bodyA = bodyA;
        rows[i].bodyB = bodyB;
        if (i < saved.size())
        {
            rows[i].impulse = saved[i];
        }
    }
}

void Constraint::carryImpulses() {}

bool Constraint::hasProjectedRows() const
{
    for (const ConstraintRow& row : rows)
    {
        if (row.project)
        {
            return true;
        }
    }
    return false;
}

void Constraint::warmStart()
{
    for (ConstraintRow& row : rows)
    {
        applyRowImpulse(row, row.impulse);
    }
}

void Constraint::solveVelocity()
{
    for (ConstraintRow& row : rows)
    {
        solveRow(row);
    }
}

} // namespace sim::physics
