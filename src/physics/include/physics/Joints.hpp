// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/Constraint.hpp"
#include "physics/PhysicsTypes.hpp"

namespace sim::physics {

// Keeps two anchor points at a fixed distance. One row.
class DistanceConstraint : public Constraint
{
public:
    Vec3 localAnchorA{0.0};
    Vec3 localAnchorB{0.0};
    Real distance = 1.0;
    SoftParams soft{60.0, 1.0};

    void prepare(Real dt) override;
    const char* typeName() const override { return "Distance"; }
};

// A ball / spherical joint: the two anchor points coincide. Three rows.
class SphericalConstraint : public Constraint
{
public:
    Vec3 localAnchorA{0.0};
    Vec3 localAnchorB{0.0};
    SoftParams soft{60.0, 1.0};

    void prepare(Real dt) override;
    const char* typeName() const override { return "Spherical"; }
};

// Welds two bodies: anchors coincide and relative orientation is locked. Six
// rows. Call captureReference() once after the bodies are in place.
class FixedConstraint : public Constraint
{
public:
    Vec3 localAnchorA{0.0};
    Vec3 localAnchorB{0.0};
    Quat referenceRotation{1.0, 0.0, 0.0, 0.0};
    SoftParams soft{60.0, 1.0};

    void captureReference();
    void prepare(Real dt) override;
    const char* typeName() const override { return "Fixed"; }
};

// A revolute / hinge joint: anchors coincide, rotation is free about one axis,
// with optional motor and angle limits about that axis. Seven rows.
class RevoluteConstraint : public Constraint
{
public:
    Vec3 localAnchorA{0.0};
    Vec3 localAnchorB{0.0};
    Vec3 localAxisA{0.0, 1.0, 0.0};
    Quat referenceRotation{1.0, 0.0, 0.0, 0.0};
    SoftParams soft{60.0, 1.0};

    bool motorEnabled = false;
    Real motorSpeed = 0.0;       // rad/s target
    Real maxMotorTorque = 0.0;   // N*m

    bool limitsEnabled = false;
    Real lowerAngle = 0.0;  // rad
    Real upperAngle = 0.0;  // rad
    SoftParams limitSoft{30.0, 1.0};

    void captureReference();
    Real angle() const;  // current twist about the hinge axis, relative to reference
    void prepare(Real dt) override;
    const char* typeName() const override { return "Revolute"; }

private:
    Quat relativeError() const;
};

// A prismatic / slider joint: relative rotation is locked, translation is free
// along one axis, with optional motor and translation limits. Seven rows.
class PrismaticConstraint : public Constraint
{
public:
    Vec3 localAnchorA{0.0};
    Vec3 localAnchorB{0.0};
    Vec3 localAxisA{1.0, 0.0, 0.0};
    Quat referenceRotation{1.0, 0.0, 0.0, 0.0};
    Real referenceTranslation = 0.0;
    SoftParams soft{60.0, 1.0};

    bool motorEnabled = false;
    Real motorSpeed = 0.0;     // m/s target
    Real maxMotorForce = 0.0;  // N

    bool limitsEnabled = false;
    Real lowerTranslation = 0.0;  // m, relative to reference
    Real upperTranslation = 0.0;
    SoftParams limitSoft{30.0, 1.0};

    void captureReference();
    Real translation() const;
    void prepare(Real dt) override;
    const char* typeName() const override { return "Prismatic"; }
};

} // namespace sim::physics
