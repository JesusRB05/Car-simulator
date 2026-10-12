// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Integrator.hpp"

namespace sim::physics::integrator {

void integrateVelocities(RigidBody& body, const Vec3& gravity, Real dt)
{
    if (!body.isDynamic())
    {
        return;
    }

    body.linearVelocity += (gravity + body.forceAccumulator * body.inverseMass) * dt;
    if (body.linearDamping > 0.0)
    {
        body.linearVelocity *= 1.0 / (1.0 + body.linearDamping * dt);
    }

    // Gyroscopic term handled with the implicit midpoint rule. The world-space
    // inertia is frozen for the step, so the update is a fixed-point problem
    //
    //   omega_new = omega_old + dt * Iinv * (torque - omega_mid x (I * omega_mid))
    //
    // with omega_mid = (omega_old + omega_new) / 2. The midpoint scheme is
    // symplectic and preserves every quadratic invariant of this system exactly,
    // in particular the kinetic energy and the angular-momentum magnitude, so a
    // fast wheel neither gains energy nor dampens its spin.
    const Mat3 inertiaWorld = body.rotationCache * body.inertiaLocal * glm::transpose(body.rotationCache);
    const Mat3 inverseInertiaWorld = body.inverseInertiaWorldCache;
    const Vec3 angularOld = body.angularVelocity;
    Vec3 angularNew = angularOld;
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        const Vec3 angularMid = 0.5 * (angularOld + angularNew);
        const Vec3 gyroscopic = glm::cross(angularMid, inertiaWorld * angularMid);
        angularNew = angularOld + dt * (inverseInertiaWorld * (body.torqueAccumulator - gyroscopic));
    }
    body.angularVelocity = angularNew;

    if (body.angularDamping > 0.0)
    {
        body.angularVelocity *= 1.0 / (1.0 + body.angularDamping * dt);
    }
}

void integratePositions(RigidBody& body, Real dt)
{
    if (body.isStatic())
    {
        return;
    }

    body.position += body.linearVelocity * dt;

    const Vec3 rotationVector = body.angularVelocity * dt;
    body.orientation = quaternionNormalize(quaternionMultiply(quaternionFromRotationVector(rotationVector), body.orientation));

    body.updateCache();
    body.clearAccumulators();
}

void integrateBody(RigidBody& body, const Vec3& gravity, Real dt)
{
    integrateVelocities(body, gravity, dt);
    integratePositions(body, dt);
}

} // namespace sim::physics::integrator
