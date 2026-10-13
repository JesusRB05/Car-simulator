// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/RigidBody.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace sim::physics {
namespace {

// Classic cyclic Jacobi eigenvalue iteration for a symmetric 3x3 matrix. Small
// enough that the sweeps cost nothing; used only for validation and for
// extracting principal moments from a general inertia tensor.
Vec3 jacobiEigenvalues(const Mat3& input, Mat3* eigenvectors) {
    double a[3][3];
    double v[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = input[i][j];
            v[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int sweep = 0; sweep < 64; ++sweep) {
        const double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (off < 1e-18) {
            break;
        }
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                const double apq = a[p][q];
                if (std::abs(apq) < 1e-300) {
                    continue;
                }
                const double theta = (a[q][q] - a[p][p]) / (2.0 * apq);
                double t = 0.0;
                if (theta >= 0.0) {
                    t = 1.0 / (theta + std::sqrt(theta * theta + 1.0));
                } else {
                    t = -1.0 / (-theta + std::sqrt(theta * theta + 1.0));
                }
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                const double app = a[p][p];
                const double aqq = a[q][q];

                for (int i = 0; i < 3; ++i) {
                    const double aip = a[i][p];
                    const double aiq = a[i][q];
                    a[i][p] = c * aip - s * aiq;
                    a[i][q] = s * aip + c * aiq;
                }
                a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
                a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
                a[p][q] = 0.0;
                a[q][p] = 0.0;

                for (int i = 0; i < 3; ++i) {
                    const double vip = v[i][p];
                    const double viq = v[i][q];
                    v[i][p] = c * vip - s * viq;
                    v[i][q] = s * vip + c * viq;
                }
            }
        }
    }

    std::array<int, 3> order{0, 1, 2};
    std::array<double, 3> eval{a[0][0], a[1][1], a[2][2]};
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) { return eval[lhs] < eval[rhs]; });

    if (eigenvectors != nullptr) {
        *eigenvectors = Mat3(v[0][order[0]], v[1][order[0]], v[2][order[0]], v[0][order[1]], v[1][order[1]],
                             v[2][order[1]], v[0][order[2]], v[1][order[2]], v[2][order[2]]);
    }
    return Vec3(eval[order[0]], eval[order[1]], eval[order[2]]);
}

} // namespace

Vec3 symmetricEigenvalues(const Mat3& symmetric, Mat3* eigenvectors) {
    return jacobiEigenvalues(symmetric, eigenvectors);
}

Shape Shape::box(const Vec3& halfExtents) {
    Shape shape;
    shape.type = ShapeType::Box;
    shape.halfExtents = halfExtents;
    return shape;
}

Shape Shape::sphere(Real radius) {
    Shape shape;
    shape.type = ShapeType::Sphere;
    shape.radius = radius;
    return shape;
}

Shape Shape::cylinder(Real radius, Real halfHeight) {
    Shape shape;
    shape.type = ShapeType::Cylinder;
    shape.radius = radius;
    shape.halfHeight = halfHeight;
    return shape;
}

Shape Shape::terrain(std::shared_ptr<const Heightfield> field) {
    Shape shape;
    shape.type = ShapeType::Heightfield;
    shape.heightfield = std::move(field);
    return shape;
}

Vec3 shapeInertia(const Shape& shape, Real mass) {
    if (mass <= 0.0) {
        return Vec3(0.0);
    }
    switch (shape.type) {
    case ShapeType::Box: {
        const Vec3 d = 2.0 * shape.halfExtents;
        return Vec3(mass / 12.0 * (d.y * d.y + d.z * d.z), mass / 12.0 * (d.x * d.x + d.z * d.z),
                    mass / 12.0 * (d.x * d.x + d.y * d.y));
    }
    case ShapeType::Sphere: {
        const Real i = 0.4 * mass * shape.radius * shape.radius;
        return Vec3(i);
    }
    case ShapeType::Cylinder: {
        const Real r2 = shape.radius * shape.radius;
        const Real h2 = 4.0 * shape.halfHeight * shape.halfHeight;
        const Real axis = 0.5 * mass * r2;
        const Real transverse = mass / 12.0 * (3.0 * r2 + h2);
        return Vec3(transverse, axis, transverse);
    }
    case ShapeType::Heightfield:
        return Vec3(0.0);
    }
    return Vec3(1.0);
}

void RigidBody::setMass(Real newMass) {
    mass = newMass;
    inverseMass = (newMass > 0.0 && type == BodyType::Dynamic) ? 1.0 / newMass : 0.0;
}

void RigidBody::setInertiaPrincipal(const Vec3& moments) {
    principalInertia = moments;
    inertiaLocal = Mat3(moments.x, 0.0, 0.0, 0.0, moments.y, 0.0, 0.0, 0.0, moments.z);
    if (type == BodyType::Dynamic) {
        const Vec3 safe = glm::max(moments, Vec3(1e-12));
        inverseInertiaLocal = Mat3(1.0 / safe.x, 0.0, 0.0, 0.0, 1.0 / safe.y, 0.0, 0.0, 0.0, 1.0 / safe.z);
    } else {
        inverseInertiaLocal = Mat3(0.0);
    }
}

void RigidBody::setInertiaTensor(const Mat3& tensor) {
    inertiaLocal = tensor;
    principalInertia = symmetricEigenvalues(tensor);
    if (type == BodyType::Dynamic) {
        inverseInertiaLocal = glm::inverse(tensor);
    } else {
        inverseInertiaLocal = Mat3(0.0);
    }
}

void RigidBody::setShapeInertia() {
    setInertiaPrincipal(shapeInertia(shape, mass));
}

void RigidBody::setType(BodyType newType) {
    type = newType;
    if (newType == BodyType::Dynamic) {
        inverseMass = mass > 0.0 ? 1.0 / mass : 0.0;
        const Vec3 safe = glm::max(principalInertia, Vec3(1e-12));
        inverseInertiaLocal = Mat3(1.0 / safe.x, 0.0, 0.0, 0.0, 1.0 / safe.y, 0.0, 0.0, 0.0, 1.0 / safe.z);
    } else {
        inverseMass = 0.0;
        inverseInertiaLocal = Mat3(0.0);
    }
}

bool RigidBody::validateInertia(std::string* reason) const {
    const Mat3& m = inertiaLocal;
    const Real tolerance = 1e-9;
    if (std::abs(m[0][1] - m[1][0]) > tolerance || std::abs(m[0][2] - m[2][0]) > tolerance ||
        std::abs(m[1][2] - m[2][1]) > tolerance) {
        if (reason != nullptr) {
            *reason = "inertia tensor is not symmetric";
        }
        return false;
    }

    const Vec3 moments = symmetricEigenvalues(m);
    if (moments.x <= 0.0 || moments.y <= 0.0 || moments.z <= 0.0) {
        if (reason != nullptr) {
            *reason = "inertia tensor is not positive definite";
        }
        return false;
    }
    if (moments.z > moments.x + moments.y + 1e-9) {
        if (reason != nullptr) {
            *reason = "principal moments violate the triangle inequality";
        }
        return false;
    }
    return true;
}

bool RigidBody::isFiniteState() const {
    return isFinite(position) && isFinite(orientation) && isFinite(linearVelocity) && isFinite(angularVelocity) &&
           std::isfinite(mass) && std::isfinite(inverseMass);
}

void RigidBody::updateCache() {
    rotationCache = rotationMatrix(orientation);
    inverseInertiaWorldCache = rotationCache * inverseInertiaLocal * glm::transpose(rotationCache);
}

void RigidBody::applyForceAtWorldPoint(const Vec3& force, const Vec3& worldPoint) {
    forceAccumulator += force;
    torqueAccumulator += glm::cross(worldPoint - position, force);
}

void RigidBody::applyForceAtLocalPoint(const Vec3& force, const Vec3& localPoint) {
    applyForceAtWorldPoint(force, localToWorldPoint(localPoint));
}

void RigidBody::applyCentralImpulse(const Vec3& impulse) {
    wake();
    linearVelocity += impulse * inverseMass;
}

void RigidBody::applyImpulseAtWorldPoint(const Vec3& impulse, const Vec3& worldPoint) {
    wake();
    linearVelocity += impulse * inverseMass;
    angularVelocity += inverseInertiaWorld() * glm::cross(worldPoint - position, impulse);
}

void RigidBody::applyAngularImpulse(const Vec3& impulse) {
    wake();
    angularVelocity += inverseInertiaWorld() * impulse;
}

void RigidBody::clearAccumulators() {
    forceAccumulator = Vec3(0.0);
    torqueAccumulator = Vec3(0.0);
}

Vec3 RigidBody::velocityAtWorldPoint(const Vec3& worldPoint) const {
    return linearVelocity + glm::cross(angularVelocity, worldPoint - position);
}

Real RigidBody::kineticEnergy() const {
    if (!isDynamic()) {
        return 0.0;
    }
    const Mat3 worldInertia = rotationCache * inertiaLocal * glm::transpose(rotationCache);
    const Real linear = 0.5 * mass * glm::dot(linearVelocity, linearVelocity);
    const Real angular = 0.5 * glm::dot(angularVelocity, worldInertia * angularVelocity);
    return linear + angular;
}

Vec3 RigidBody::angularMomentumAbout(const Vec3& referencePoint) const {
    if (!isDynamic()) {
        return Vec3(0.0);
    }
    const Mat3 worldInertia = rotationCache * inertiaLocal * glm::transpose(rotationCache);
    return mass * glm::cross(position - referencePoint, linearVelocity) + worldInertia * angularVelocity;
}

} // namespace sim::physics
