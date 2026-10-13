// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>

#include "physics/Curve.hpp"
#include "physics/PhysicsTypes.hpp"
#include "physics/RigidBody.hpp"

namespace sim::physics {

// Base class for every force-producing element registered with the world. An
// element only adds to the force/torque accumulators of the bodies it touches;
// the integrator consumes those accumulators afterwards.
class ForceElement {
public:
    virtual ~ForceElement() = default;

    virtual void apply() = 0;
    virtual const char* typeName() const = 0;

    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr; // may be null: the anchor is fixed in the world
    std::string name;
};

// A point-to-point linear (or curved) spring-damper. If `bodyB` is null the B
// anchor is the fixed world point `fixedAnchorB`.
class PointSpringDamper : public ForceElement {
public:
    Vec3 localAnchorA{0.0};
    Vec3 localAnchorB{0.0};
    Vec3 fixedAnchorB{0.0};

    Real restLength = 1.0;
    Real stiffness = 100.0;
    Real damping = 0.0;
    Curve forceCurve; // when non-empty, overrides stiffness with f(extension)

    void apply() override;
    const char* typeName() const override { return "PointSpringDamper"; }
};

// A torsional spring-damper about a local axis. It drives the relative twist of
// bodyB with respect to bodyA (measured about the axis) towards `restAngle`.
class RotationalSpringDamper : public ForceElement {
public:
    Vec3 localAxisA{0.0, 1.0, 0.0};

    Real restAngle = 0.0;
    Real stiffness = 10.0;
    Real damping = 0.0;
    Curve torqueCurve; // when non-empty, overrides stiffness with f(angle - restAngle)

    void apply() override;
    const char* typeName() const override { return "RotationalSpringDamper"; }
};

} // namespace sim::physics
