// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"
#include "physics/RigidBody.hpp"

namespace sim::physics {

// Semi-implicit (symplectic) Euler integrator specialised for rigid bodies.
//
// Velocities are advanced first, then positions, so the scheme is stable for
// the stiff wheel and suspension regimes the vehicle model needs. The angular
// velocity update runs the gyroscopic term through the implicit midpoint rule,
//
//   omega_new = omega_old + dt * Iinv * (tau - omega_mid x (I * omega_mid))
//
// with omega_mid = (omega_old + omega_new) / 2 (solved by a short fixed-point
// loop). The midpoint rule preserves every quadratic invariant of the frozen
// step exactly, so the kinetic energy and the angular-momentum magnitude of a
// fast-spinning body are conserved: the wheel neither gains energy nor bleeds
// its spin down.
namespace integrator {

// Advances linear and angular velocity by dt, applying gravity, the accumulated
// forces/torques and optional damping. Requires body.updateCache() to have run.
void integrateVelocities(RigidBody& body, const Vec3& gravity, Real dt);

// Advances position and orientation by dt using an exponential-map quaternion
// update, then refreshes the cache and clears the force/torque accumulators.
void integratePositions(RigidBody& body, Real dt);

// Convenience: integrateVelocities followed by integratePositions.
void integrateBody(RigidBody& body, const Vec3& gravity, Real dt);

} // namespace integrator

} // namespace sim::physics
