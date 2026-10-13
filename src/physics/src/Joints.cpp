// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Joints.hpp"

namespace sim::physics {
namespace {

const Vec3 kAxes[3] = {Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0), Vec3(0.0, 0.0, 1.0)};

void buildPointRow(ConstraintRow& row, const Vec3& e, const Vec3& rA, const Vec3& rB, Real error, Real dt,
                   const SoftParams& soft) {
    row.linearA = e;
    row.linearB = -e;
    row.angularA = glm::cross(rA, e);
    row.angularB = -glm::cross(rB, e);
    row.velocityTarget = 0.0;
    configureRow(row, constraintMass(row), error, dt, soft);
}

void buildAngularRow(ConstraintRow& row, const Vec3& e, Real error, Real dt, const SoftParams& soft) {
    row.linearA = Vec3(0.0);
    row.linearB = Vec3(0.0);
    row.angularA = -e;
    row.angularB = e;
    row.velocityTarget = 0.0;
    configureRow(row, constraintMass(row), error, dt, soft);
}

void disableRow(ConstraintRow& row) {
    row.effectiveMass = 0.0;
    row.softness = 0.0;
    row.bias = 0.0;
    row.impulse = 0.0;
    row.lower = 0.0;
    row.upper = 0.0;
}

Quat relativeToReference(const Constraint& constraint, const Quat& reference) {
    const Quat relative =
        quaternionMultiply(quaternionInverse(constraint.bodyA->orientation), constraint.bodyB->orientation);
    return quaternionMultiply(relative, quaternionInverse(reference));
}

} // namespace

void DistanceConstraint::prepare(Real dt) {
    rebuildRows(1);

    const Vec3 pA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pB = bodyB->localToWorldPoint(localAnchorB);
    const Vec3 rA = pA - bodyA->position;
    const Vec3 rB = pB - bodyB->position;
    const Vec3 delta = pA - pB;
    const Real length = glm::length(delta);
    const Vec3 direction = safeNormalize(delta, Vec3(1.0, 0.0, 0.0));

    ConstraintRow& row = rows[0];
    row.linearA = direction;
    row.linearB = -direction;
    row.angularA = glm::cross(rA, direction);
    row.angularB = -glm::cross(rB, direction);
    row.velocityTarget = 0.0;
    configureRow(row, constraintMass(row), length - distance, dt, soft);
}

void SphericalConstraint::prepare(Real dt) {
    rebuildRows(3);

    const Vec3 pA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pB = bodyB->localToWorldPoint(localAnchorB);
    const Vec3 rA = pA - bodyA->position;
    const Vec3 rB = pB - bodyB->position;
    const Vec3 error = pA - pB;

    for (int i = 0; i < 3; ++i) {
        buildPointRow(rows[static_cast<std::size_t>(i)], kAxes[i], rA, rB, glm::dot(error, kAxes[i]), dt, soft);
    }
}

void FixedConstraint::captureReference() {
    referenceRotation = quaternionMultiply(quaternionInverse(bodyA->orientation), bodyB->orientation);
}

void FixedConstraint::prepare(Real dt) {
    rebuildRows(6);

    const Vec3 pA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pB = bodyB->localToWorldPoint(localAnchorB);
    const Vec3 rA = pA - bodyA->position;
    const Vec3 rB = pB - bodyB->position;
    const Vec3 error = pA - pB;

    for (int i = 0; i < 3; ++i) {
        buildPointRow(rows[static_cast<std::size_t>(i)], kAxes[i], rA, rB, glm::dot(error, kAxes[i]), dt, soft);
    }

    const Quat qErr = relativeToReference(*this, referenceRotation);
    const Vec3 errorWorld = rotateVector(bodyA->orientation, quaternionRotationVector(qErr));
    for (int i = 0; i < 3; ++i) {
        buildAngularRow(rows[static_cast<std::size_t>(3 + i)], kAxes[i], glm::dot(errorWorld, kAxes[i]), dt, soft);
    }
}

void RevoluteConstraint::captureReference() {
    referenceRotation = quaternionMultiply(quaternionInverse(bodyA->orientation), bodyB->orientation);
}

Quat RevoluteConstraint::relativeError() const {
    return relativeToReference(*this, referenceRotation);
}

Real RevoluteConstraint::angle() const {
    return glm::dot(quaternionRotationVector(relativeError()), localAxisA);
}

void RevoluteConstraint::prepare(Real dt) {
    rebuildRows(7);

    const Vec3 pA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pB = bodyB->localToWorldPoint(localAnchorB);
    const Vec3 rA = pA - bodyA->position;
    const Vec3 rB = pB - bodyB->position;
    const Vec3 error = pA - pB;

    for (int i = 0; i < 3; ++i) {
        buildPointRow(rows[static_cast<std::size_t>(i)], kAxes[i], rA, rB, glm::dot(error, kAxes[i]), dt, soft);
    }

    const Vec3 errorLocal = quaternionRotationVector(relativeError());
    const Vec3 perpendicularLocal = errorLocal - glm::dot(errorLocal, localAxisA) * localAxisA;
    const Vec3 errorWorld = rotateVector(bodyA->orientation, perpendicularLocal);

    const Vec3 axisWorld = safeNormalize(rotateVector(bodyA->orientation, localAxisA), Vec3(0.0, 1.0, 0.0));
    Vec3 u;
    Vec3 v;
    perpendicularBasis(axisWorld, u, v);

    buildAngularRow(rows[3], u, glm::dot(errorWorld, u), dt, soft);
    buildAngularRow(rows[4], v, glm::dot(errorWorld, v), dt, soft);

    const Real twist = glm::dot(errorLocal, localAxisA);
    const Real velocityTarget = motorSpeed;

    // Motor row (index 5).
    ConstraintRow& motor = rows[5];
    if (motorEnabled) {
        motor.linearA = Vec3(0.0);
        motor.linearB = Vec3(0.0);
        motor.angularA = -axisWorld;
        motor.angularB = axisWorld;
        motor.velocityTarget = velocityTarget;
        configureRow(motor, constraintMass(motor), 0.0, dt, SoftParams{0.0, 1.0});
        motor.lower = -maxMotorTorque * dt;
        motor.upper = maxMotorTorque * dt;
    } else {
        disableRow(motor);
    }

    // Limit row (index 6).
    ConstraintRow& limit = rows[6];
    limit.linearA = Vec3(0.0);
    limit.linearB = Vec3(0.0);
    limit.angularA = -axisWorld;
    limit.angularB = axisWorld;
    limit.velocityTarget = 0.0;
    if (limitsEnabled && twist < lowerAngle) {
        configureRow(limit, constraintMass(limit), twist - lowerAngle, dt, limitSoft);
        limit.lower = 0.0;
        limit.upper = kInfinity;
    } else if (limitsEnabled && twist > upperAngle) {
        configureRow(limit, constraintMass(limit), twist - upperAngle, dt, limitSoft);
        limit.lower = -kInfinity;
        limit.upper = 0.0;
    } else {
        disableRow(limit);
    }
}

void PrismaticConstraint::captureReference() {
    referenceRotation = quaternionMultiply(quaternionInverse(bodyA->orientation), bodyB->orientation);
    const Vec3 pA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pB = bodyB->localToWorldPoint(localAnchorB);
    const Vec3 axisWorld = safeNormalize(rotateVector(bodyA->orientation, localAxisA), Vec3(1.0, 0.0, 0.0));
    referenceTranslation = glm::dot(pA - pB, axisWorld);
}

Real PrismaticConstraint::translation() const {
    const Vec3 pA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pB = bodyB->localToWorldPoint(localAnchorB);
    const Vec3 axisWorld = safeNormalize(rotateVector(bodyA->orientation, localAxisA), Vec3(1.0, 0.0, 0.0));
    return glm::dot(pA - pB, axisWorld) - referenceTranslation;
}

void PrismaticConstraint::prepare(Real dt) {
    rebuildRows(7);

    const Vec3 pA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pB = bodyB->localToWorldPoint(localAnchorB);
    const Vec3 rA = pA - bodyA->position;
    const Vec3 rB = pB - bodyB->position;
    const Vec3 error = pA - pB;

    const Vec3 axisWorld = safeNormalize(rotateVector(bodyA->orientation, localAxisA), Vec3(1.0, 0.0, 0.0));
    Vec3 u;
    Vec3 v;
    perpendicularBasis(axisWorld, u, v);

    buildPointRow(rows[0], u, rA, rB, glm::dot(error, u), dt, soft);
    buildPointRow(rows[1], v, rA, rB, glm::dot(error, v), dt, soft);

    const Quat qErr = relativeToReference(*this, referenceRotation);
    const Vec3 errorWorld = rotateVector(bodyA->orientation, quaternionRotationVector(qErr));
    buildAngularRow(rows[2], kAxes[0], glm::dot(errorWorld, kAxes[0]), dt, soft);
    buildAngularRow(rows[3], kAxes[1], glm::dot(errorWorld, kAxes[1]), dt, soft);
    buildAngularRow(rows[4], kAxes[2], glm::dot(errorWorld, kAxes[2]), dt, soft);

    const Real coordinate = glm::dot(pA - pB, axisWorld) - referenceTranslation;

    ConstraintRow& motor = rows[5];
    if (motorEnabled) {
        motor.linearA = axisWorld;
        motor.linearB = -axisWorld;
        motor.angularA = glm::cross(rA, axisWorld);
        motor.angularB = -glm::cross(rB, axisWorld);
        motor.velocityTarget = motorSpeed;
        configureRow(motor, constraintMass(motor), 0.0, dt, SoftParams{0.0, 1.0});
        motor.lower = -maxMotorForce * dt;
        motor.upper = maxMotorForce * dt;
    } else {
        disableRow(motor);
    }

    ConstraintRow& limit = rows[6];
    limit.linearA = axisWorld;
    limit.linearB = -axisWorld;
    limit.angularA = glm::cross(rA, axisWorld);
    limit.angularB = -glm::cross(rB, axisWorld);
    limit.velocityTarget = 0.0;
    if (limitsEnabled && coordinate < lowerTranslation) {
        configureRow(limit, constraintMass(limit), coordinate - lowerTranslation, dt, limitSoft);
        limit.lower = 0.0;
        limit.upper = kInfinity;
    } else if (limitsEnabled && coordinate > upperTranslation) {
        configureRow(limit, constraintMass(limit), coordinate - upperTranslation, dt, limitSoft);
        limit.lower = -kInfinity;
        limit.upper = 0.0;
    } else {
        disableRow(limit);
    }
}

} // namespace sim::physics
