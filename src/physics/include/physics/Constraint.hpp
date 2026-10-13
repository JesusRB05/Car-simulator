// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

#include "physics/PhysicsTypes.hpp"
#include "physics/RigidBody.hpp"

namespace sim::physics {

// A single scalar constraint row in maximal coordinates. All directions are in
// world space. The row represents  J * v_target = 0  where the constraint
// velocity is
//
//   Cdot = dot(linearA, vA) + dot(angularA, wA)
//        + dot(linearB, vB) + dot(angularB, wB)
//
// An impulse `lambda` applied along the row changes the body velocities by
//
//   vA += invMassA * linearA  * lambda,  wA += invInertiaA * angularA  * lambda
//   vB += invMassB * linearB  * lambda,  wB += invInertiaB * angularB  * lambda
struct ConstraintRow {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Vec3 linearA{0.0};
    Vec3 angularA{0.0};
    Vec3 linearB{0.0};
    Vec3 angularB{0.0};

    Real velocityTarget = 0.0; // Cdot is driven towards this value
    Real bias = 0.0;           // soft-constraint position term
    Real softness = 0.0;       // gamma (CFM)
    Real effectiveMass = 0.0;  // 1 / (K + gamma)
    Real impulse = 0.0;        // accumulated impulse (warm started)
    Real lower = -kInfinity;   // impulse clamp (limits / motors)
    Real upper = kInfinity;

    Real positionError = 0.0; // scalar position error C at prepare() time
    bool project = false;     // rigid row: corrected by the position projection pass

    bool isBilateral() const { return lower == -kInfinity && upper == kInfinity; }
};

// Per-joint compliance. Three modes, selected in this order:
//
//  * Rigid (`rigid == true` or `stiffness >= 1e18`): the velocity rows have no
//    softness (gamma = 0) and no bias, so the velocity solve injects no energy,
//    and the world removes the remaining position error with a projection pass
//    after the positions are integrated (see PhysicsWorld::step). This is the
//    mode to use when the joint must not visibly stretch.
//  * Soft step (`stiffness > 0` or `frequency > 0`): an implicit spring-damper
//    that enforces the velocity constraint and gently removes position error,
//    without the energy injection of an aggressive Baumgarte term. An explicit
//    `stiffness` (N/m or N*m/rad) overrides the frequency-derived one.
//  * Velocity-only (`frequency <= 0`, `stiffness <= 0`, not rigid): gamma = 0
//    and no position correction at all. Only meant for motor rows, whose target
//    is a velocity; a positional joint configured like this drifts.
struct SoftParams {
    Real frequency = 0.0;    // Hz
    Real dampingRatio = 1.0; // 1 = critically damped
    Real stiffness = 0.0;    // N/m or N*m/rad (0 = use frequency-based stiffness)
    bool rigid = false;      // rigid row + position projection

    bool isRigid() const { return rigid || stiffness >= 1e18; }

    static SoftParams makeRigid() {
        SoftParams params;
        params.rigid = true;
        return params;
    }
};

// Computes the row's constraint mass  K = J * M^-1 * J^T.
Real constraintMass(const ConstraintRow& row);

// Fills effectiveMass / softness / bias from the constraint mass K and the
// scalar position error C (C = 0 means satisfied).
void configureRow(ConstraintRow& row, Real invEffectiveMass, Real positionError, Real dt, const SoftParams& soft);

// Applies a raw impulse along the row (used for warm starting).
void applyRowImpulse(ConstraintRow& row, Real lambda);

// One sequential-impulse (PGS) update of the row.
void solveRow(ConstraintRow& row);

// Base class of every joint. Concrete joints build their rows in prepare(),
// then the world warm-starts and iterates them.
class Constraint {
public:
    virtual ~Constraint() = default;

    virtual void prepare(Real dt) = 0;
    virtual const char* typeName() const = 0;

    void warmStart();
    void solveVelocity();

    // True when any row was configured as rigid by the last prepare(), i.e. the
    // joint takes part in the position projection pass.
    bool hasProjectedRows() const;

    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    std::string name;

    std::vector<ConstraintRow> rows;

protected:
    // Rebuilds `rows` while preserving the accumulated impulses (warm start).
    void rebuildRows(std::size_t count);
    void carryImpulses();
};

} // namespace sim::physics
