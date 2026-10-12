// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Pure, dependency-free rigid-body maths used across the physics engine. Every
// routine here is a plain function of its arguments (no hidden state) and works
// in double precision, matching the engine's world-space convention.

#include <cmath>
#include <limits>

#include "physics/PhysicsTypes.hpp"

namespace sim::physics {

inline constexpr Real kInfinity = std::numeric_limits<Real>::infinity();
inline constexpr Real kPiPhysics = 3.14159265358979323846;
inline constexpr Real kTwoPiPhysics = 2.0 * kPiPhysics;

// The skew-symmetric matrix S(v) such that S(v) * w == cross(v, w).
inline Mat3 skew(const Vec3& v)
{
    return Mat3(0.0, v.z, -v.y, -v.z, 0.0, v.x, v.y, -v.x, 0.0);
}

inline Mat3 rotationMatrix(const Quat& q)
{
    return glm::mat3_cast(q);
}

inline Quat quaternionFromAxisAngle(const Vec3& axis, Real angle)
{
    const Real length = glm::length(axis);
    if (length < 1e-12)
    {
        return Quat(1.0, 0.0, 0.0, 0.0);
    }
    const Vec3 unit = axis / length;
    const Real half = 0.5 * angle;
    const Real s = std::sin(half);
    return Quat(std::cos(half), unit.x * s, unit.y * s, unit.z * s);
}

// Exponential map: turns a rotation vector (axis * angle) into a quaternion.
// Exact for a constant angular velocity over the integration interval, which is
// what keeps fast-spinning wheels accurate.
inline Quat quaternionFromRotationVector(const Vec3& rotationVector)
{
    const Real angle = glm::length(rotationVector);
    if (angle < 1e-12)
    {
        return Quat(1.0, 0.5 * rotationVector.x, 0.5 * rotationVector.y, 0.5 * rotationVector.z);
    }
    const Vec3 axis = rotationVector / angle;
    const Real half = 0.5 * angle;
    const Real s = std::sin(half);
    return Quat(std::cos(half), axis.x * s, axis.y * s, axis.z * s);
}

inline Quat quaternionMultiply(const Quat& a, const Quat& b)
{
    return Quat(a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
}

inline Quat quaternionConjugate(const Quat& q)
{
    return Quat(q.w, -q.x, -q.y, -q.z);
}

inline Quat quaternionInverse(const Quat& q)
{
    const Real normSquared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (normSquared < 1e-30)
    {
        return Quat(1.0, 0.0, 0.0, 0.0);
    }
    const Real inverse = 1.0 / normSquared;
    return Quat(q.w * inverse, -q.x * inverse, -q.y * inverse, -q.z * inverse);
}

inline Quat quaternionNormalize(const Quat& q)
{
    const Real norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 1e-30)
    {
        return Quat(1.0, 0.0, 0.0, 0.0);
    }
    const Real inverse = 1.0 / norm;
    return Quat(q.w * inverse, q.x * inverse, q.y * inverse, q.z * inverse);
}

inline Vec3 rotateVector(const Quat& q, const Vec3& v)
{
    const Vec3 u(q.x, q.y, q.z);
    const Vec3 t = 2.0 * glm::cross(u, v);
    return v + q.w * t + glm::cross(u, t);
}

// dq/dt = 0.5 * omega_quat * q, with omega expressed in the same frame as q.
inline Quat quaternionDerivative(const Quat& q, const Vec3& omega)
{
    return 0.5 * quaternionMultiply(Quat(0.0, omega.x, omega.y, omega.z), q);
}

// The rotation vector (axis * angle) of a quaternion, robust near identity.
inline Vec3 quaternionRotationVector(const Quat& q)
{
    Quat qn = q;
    if (qn.w < 0.0)
    {
        qn = Quat(-qn.w, -qn.x, -qn.y, -qn.z);
    }
    const Vec3 v(qn.x, qn.y, qn.z);
    const Real sinHalf = glm::length(v);
    if (sinHalf < 1e-12)
    {
        return 2.0 * v;
    }
    const Real angle = 2.0 * std::atan2(sinHalf, qn.w);
    return v * (angle / sinHalf);
}

struct Transform
{
    Vec3 position{0.0};
    Quat orientation{1.0, 0.0, 0.0, 0.0};
};

inline Vec3 transformPoint(const Transform& transform, const Vec3& point)
{
    return transform.position + rotateVector(transform.orientation, point);
}

inline Vec3 transformDirection(const Transform& transform, const Vec3& direction)
{
    return rotateVector(transform.orientation, direction);
}

inline Vec3 inverseTransformPoint(const Transform& transform, const Vec3& point)
{
    return rotateVector(quaternionConjugate(transform.orientation), point - transform.position);
}

inline Vec3 inverseTransformDirection(const Transform& transform, const Vec3& direction)
{
    return rotateVector(quaternionConjugate(transform.orientation), direction);
}

inline Vec3 safeNormalize(const Vec3& v, const Vec3& fallback = Vec3(0.0, 1.0, 0.0))
{
    const Real length = glm::length(v);
    if (length < 1e-12)
    {
        return fallback;
    }
    return v / length;
}

// Builds an orthonormal basis (u, v) perpendicular to the given unit axis.
inline void perpendicularBasis(const Vec3& axis, Vec3& u, Vec3& v)
{
    const Vec3 reference = std::abs(axis.x) < 0.9 ? Vec3(1.0, 0.0, 0.0) : Vec3(0.0, 1.0, 0.0);
    u = glm::normalize(glm::cross(axis, reference));
    v = glm::cross(axis, u);
}

inline bool isFinite(Real value)
{
    return std::isfinite(value);
}

inline bool isFinite(const Vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool isFinite(const Quat& q)
{
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

} // namespace sim::physics
