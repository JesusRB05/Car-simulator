// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <string>

#include "physics/Math.hpp"
#include "physics/PhysicsTypes.hpp"

namespace sim::physics {

// Defined in physics/Heightfield.hpp; only referenced through a shared pointer.
struct Heightfield;

enum class BodyType {
    Static,
    Kinematic,
    Dynamic,
};

enum class ShapeType {
    Box,
    Sphere,
    Cylinder,
    Heightfield,
};

// A primitive used for automatic inertia computation and, later, collisions and
// debug rendering. Dimensions are half-extents in the body frame. A `Heightfield`
// shape carries no inertia and is meant for static terrain; the field is authored
// directly in world coordinates, so the terrain body must stay at the identity
// transform.
struct Shape {
    ShapeType type = ShapeType::Box;
    Vec3 halfExtents{0.5, 0.5, 0.5};                // Box
    Real radius = 0.5;                              // Sphere, Cylinder
    Real halfHeight = 0.5;                          // Cylinder (local +Y axis)
    std::shared_ptr<const Heightfield> heightfield; // Heightfield

    static Shape box(const Vec3& halfExtents);
    static Shape sphere(Real radius);
    static Shape cylinder(Real radius, Real halfHeight);
    static Shape terrain(std::shared_ptr<const Heightfield> field);
};

// Computes the principal inertia moments of a uniform solid shape of the given
// mass, expressed in the body frame. Static/zero-mass bodies return zeros.
Vec3 shapeInertia(const Shape& shape, Real mass);

// The 3x3 symmetric eigenvalue helper used to validate arbitrary inertia
// tensors. Returns the eigenvalues in ascending order and, when requested, the
// matching orthonormal eigenvectors as columns.
Vec3 symmetricEigenvalues(const Mat3& symmetric, Mat3* eigenvectors = nullptr);

// A rigid body with full mass properties. Positions are stored in double
// precision. Velocities and the inertia tensor live in world/body space as
// documented on each accessor.
class RigidBody {
public:
    // --- Description ------------------------------------------------------
    BodyType type = BodyType::Dynamic;
    Shape shape = Shape::box({0.5, 0.5, 0.5});
    std::string name;

    // Opts the body into shape-based collision detection. Two bodies only
    // generate contacts when both have this set, so existing scenes and the
    // deterministic probe keep their behaviour until they opt in.
    bool collisionEnabled = false;

    // --- Mass properties --------------------------------------------------
    Real mass = 1.0;
    Real inverseMass = 1.0;
    Vec3 principalInertia{1.0};    // principal moments in the body frame
    Mat3 inertiaLocal{1.0};        // full body-space inertia tensor
    Mat3 inverseInertiaLocal{1.0}; // inverse of inertiaLocal

    // --- State ------------------------------------------------------------
    Vec3 position{0.0};
    Quat orientation{1.0, 0.0, 0.0, 0.0};
    Vec3 linearVelocity{0.0};
    Vec3 angularVelocity{0.0};

    // --- Accumulators -----------------------------------------------------
    Vec3 forceAccumulator{0.0};
    Vec3 torqueAccumulator{0.0};

    // --- Sleeping ---------------------------------------------------------
    // A dynamic body whose linear and angular speed stay below the world's
    // thresholds for `PhysicsWorld::sleepTime` seconds is put to sleep: its
    // velocities are zeroed, it is not integrated and the solver treats it as
    // static geometry until something wakes it (a contact or joint with a
    // moving body, an applied impulse, or a call to wake()).
    bool sleeping = false;
    Real sleepTimer = 0.0; // seconds spent below the sleep thresholds

    void wake() {
        sleeping = false;
        sleepTimer = 0.0;
    }

    // --- Material ---------------------------------------------------------
    Real linearDamping = 0.0; // per-second exponential decay rate
    Real angularDamping = 0.0;
    Real friction = 0.6;    // Coulomb coefficient, combined as sqrt(frictionA * frictionB)
    Real restitution = 0.0; // bounciness in [0, 1], combined as max(restitutionA, restitutionB)

    // --- Cached world-space values (refreshed by updateCache) -------------
    Mat3 rotationCache{1.0};
    Mat3 inverseInertiaWorldCache{1.0};

    void setMass(Real newMass);
    void setInertiaPrincipal(const Vec3& moments);
    void setInertiaTensor(const Mat3& tensor);
    void setShapeInertia(); // derive inertia from shape + mass

    void setType(BodyType newType);
    bool isStatic() const { return type == BodyType::Static; }
    bool isKinematic() const { return type == BodyType::Kinematic; }
    bool isDynamic() const { return type == BodyType::Dynamic; }

    // Symmetric positive definite and triangle inequality on the principal
    // moments (I1 + I2 >= I3). Returns false and fills `reason` on failure.
    bool validateInertia(std::string* reason = nullptr) const;
    bool isFiniteState() const;

    void updateCache();

    const Mat3& rotation() const { return rotationCache; }
    const Mat3& inverseInertiaWorld() const { return inverseInertiaWorldCache; }

    Transform transform() const { return Transform{position, orientation}; }
    Vec3 localToWorldPoint(const Vec3& localPoint) const { return position + rotateVector(orientation, localPoint); }
    Vec3 worldToLocalPoint(const Vec3& worldPoint) const {
        return rotateVector(quaternionConjugate(orientation), worldPoint - position);
    }
    Vec3 localToWorldDirection(const Vec3& localDirection) const { return rotateVector(orientation, localDirection); }

    // --- Force / impulse application --------------------------------------
    void applyCentralForce(const Vec3& force) { forceAccumulator += force; }
    void applyForceAtWorldPoint(const Vec3& force, const Vec3& worldPoint);
    void applyForceAtLocalPoint(const Vec3& force, const Vec3& localPoint);
    void applyTorque(const Vec3& torque) { torqueAccumulator += torque; }

    void applyCentralImpulse(const Vec3& impulse);
    void applyImpulseAtWorldPoint(const Vec3& impulse, const Vec3& worldPoint);
    void applyAngularImpulse(const Vec3& impulse);

    void clearAccumulators();

    Vec3 velocityAtWorldPoint(const Vec3& worldPoint) const;
    Real kineticEnergy() const;
    Vec3 angularMomentumAbout(const Vec3& referencePoint) const;
};

} // namespace sim::physics
