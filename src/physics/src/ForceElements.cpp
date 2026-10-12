// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/ForceElements.hpp"

namespace sim::physics {

void PointSpringDamper::apply()
{
    if (bodyA == nullptr)
    {
        return;
    }

    const Vec3 pointA = bodyA->localToWorldPoint(localAnchorA);
    const Vec3 pointB = bodyB != nullptr ? bodyB->localToWorldPoint(localAnchorB) : fixedAnchorB;

    const Vec3 delta = pointB - pointA;
    const Real length = glm::length(delta);
    const Vec3 direction = safeNormalize(delta);
    const Real extension = length - restLength;

    Real magnitude = forceCurve.empty() ? stiffness * extension : forceCurve.evaluate(extension);

    const Vec3 velocityA = bodyA->velocityAtWorldPoint(pointA);
    const Vec3 velocityB = bodyB != nullptr ? bodyB->velocityAtWorldPoint(pointB) : Vec3(0.0);
    magnitude += damping * glm::dot(velocityB - velocityA, direction);

    const Vec3 forceOnA = magnitude * direction;
    bodyA->applyForceAtWorldPoint(forceOnA, pointA);
    if (bodyB != nullptr)
    {
        bodyB->applyForceAtWorldPoint(-forceOnA, pointB);
    }
}

void RotationalSpringDamper::apply()
{
    if (bodyA == nullptr || bodyB == nullptr)
    {
        return;
    }

    const Vec3 axisWorld = bodyA->localToWorldDirection(localAxisA);
    const Quat relative = quaternionMultiply(quaternionInverse(bodyA->orientation), bodyB->orientation);
    const Vec3 rotationVector = quaternionRotationVector(relative);
    const Real angle = glm::dot(rotationVector, localAxisA);
    const Real error = angle - restAngle;

    Real magnitude = torqueCurve.empty() ? stiffness * error : torqueCurve.evaluate(error);
    magnitude += damping * glm::dot(bodyB->angularVelocity - bodyA->angularVelocity, axisWorld);

    bodyA->applyTorque(magnitude * axisWorld);
    bodyB->applyTorque(-magnitude * axisWorld);
}

} // namespace sim::physics
