// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Collision.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "physics/Broadphase.hpp"
#include "physics/Convex.hpp"
#include "physics/Heightfield.hpp"
#include "physics/PhysicsWorld.hpp"

namespace sim::physics {
namespace {

// Overlaps within this band are treated as equal so that a redundant edge axis
// (for example A.x cross B.z == -Y for axis-aligned boxes, which duplicates the
// Y face axis) never wins the SAT race against the face it mirrors.
constexpr Real kOverlapTie = 1e-9;

// Orientable box in world space, used by the OBB narrowphase.
struct Obb {
    Vec3 center{0.0};
    Vec3 axis[3]{Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0), Vec3(0.0, 0.0, 1.0)};
    Vec3 halfExtents{0.5, 0.5, 0.5};
};

Obb makeObb(const RigidBody& body) {
    Obb obb;
    obb.center = body.position;
    obb.halfExtents = body.shape.halfExtents;
    obb.axis[0] = body.localToWorldDirection(Vec3(1.0, 0.0, 0.0));
    obb.axis[1] = body.localToWorldDirection(Vec3(0.0, 1.0, 0.0));
    obb.axis[2] = body.localToWorldDirection(Vec3(0.0, 0.0, 1.0));
    return obb;
}

Real component(const Vec3& v, int i) {
    return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

Vec3 supportPoint(const Obb& obb, const Vec3& direction) {
    Vec3 point = obb.center;
    for (int i = 0; i < 3; ++i) {
        const Real sign = glm::dot(direction, obb.axis[i]) >= 0.0 ? 1.0 : -1.0;
        point += obb.axis[i] * (sign * component(obb.halfExtents, i));
    }
    return point;
}

int sphereSphere(const RigidBody& a, const RigidBody& b, ContactPoint* out, Real margin) {
    const Vec3 delta = b.position - a.position;
    const Real distanceSquared = glm::dot(delta, delta);
    const Real sumRadii = a.shape.radius + b.shape.radius;
    const Real reach = sumRadii + margin;
    if (distanceSquared > reach * reach) {
        return 0;
    }
    const Real distance = std::sqrt(distanceSquared);
    const Vec3 normal = distance > 1e-12 ? delta / distance : Vec3(0.0, 1.0, 0.0);
    const Real penetration = sumRadii - distance;
    out[0].normal = normal;
    out[0].penetration = penetration;
    out[0].position = a.position + normal * (a.shape.radius - 0.5 * penetration);
    return 1;
}

// Sphere (a) against box (b). The returned normal points from the sphere to the
// box.
int sphereBox(const RigidBody& a, const RigidBody& b, ContactPoint* out, Real margin) {
    const Vec3 halfExtents = b.shape.halfExtents;
    const Vec3 local = b.worldToLocalPoint(a.position);
    const Vec3 clamped = glm::clamp(local, -halfExtents, halfExtents);
    const Vec3 delta = local - clamped; // box surface -> sphere centre
    const Real distanceSquared = glm::dot(delta, delta);

    Vec3 normalLocal; // box -> sphere
    Vec3 pointLocal;
    Real penetration;
    if (distanceSquared > 1e-24) {
        const Real distance = std::sqrt(distanceSquared);
        if (distance > a.shape.radius + margin) {
            return 0;
        }
        normalLocal = delta / distance;
        pointLocal = clamped;
        penetration = a.shape.radius - distance;
    } else {
        // Sphere centre inside the box: push out along the axis of least overlap.
        int axis = 0;
        Real least = halfExtents.x - std::abs(local.x);
        for (int i = 1; i < 3; ++i) {
            const Real overlap = component(halfExtents, i) - std::abs(component(local, i));
            if (overlap < least) {
                least = overlap;
                axis = i;
            }
        }
        const Real sign = component(local, axis) >= 0.0 ? 1.0 : -1.0;
        normalLocal = Vec3(0.0);
        normalLocal[axis] = sign;
        pointLocal = local;
        pointLocal[axis] = sign * component(halfExtents, axis);
        penetration = a.shape.radius + least;
    }

    out[0].normal = -b.localToWorldDirection(normalLocal);
    out[0].position = b.localToWorldPoint(pointLocal);
    out[0].penetration = penetration;
    return 1;
}

int sphereCylinder(const RigidBody& a, const RigidBody& b, ContactPoint* out, Real margin) {
    const Real radius = a.shape.radius;
    const Real cylinderRadius = b.shape.radius;
    const Real halfHeight = b.shape.halfHeight;

    const Vec3 local = b.worldToLocalPoint(a.position);
    const Real radialLength = std::sqrt(local.x * local.x + local.z * local.z);
    const Real axial = std::clamp(local.y, -halfHeight, halfHeight);

    Vec3 closestLocal;
    if (radialLength > 1e-12) {
        const Real clampedRadius = std::min(radialLength, cylinderRadius);
        closestLocal = Vec3(local.x * (clampedRadius / radialLength), axial, local.z * (clampedRadius / radialLength));
    } else {
        closestLocal = Vec3(0.0, axial, 0.0);
    }

    const Vec3 delta = local - closestLocal; // cylinder surface -> sphere centre
    const Real distance = glm::length(delta);

    Vec3 normalLocal; // cylinder -> sphere
    Real penetration;
    if (distance > 1e-12) {
        if (distance > radius + margin) {
            return 0;
        }
        normalLocal = delta / distance;
        penetration = radius - distance;
    } else {
        // Sphere centre inside the solid cylinder: least of radial and cap gaps.
        const Real radialPenetration = cylinderRadius - radialLength;
        const Real capPenetration = halfHeight - std::abs(local.y);
        if (radialPenetration <= capPenetration) {
            const Vec3 radial(local.x, 0.0, local.z);
            normalLocal = radialLength > 1e-12 ? radial / radialLength : Vec3(1.0, 0.0, 0.0);
            penetration = radius + radialPenetration;
        } else {
            normalLocal = Vec3(0.0, local.y >= 0.0 ? 1.0 : -1.0, 0.0);
            penetration = radius + capPenetration;
        }
        closestLocal = local;
    }

    out[0].normal = -b.localToWorldDirection(normalLocal);
    out[0].position = b.localToWorldPoint(closestLocal);
    out[0].penetration = penetration;
    return 1;
}

// Clips a convex polygon (given as `count` points) against the half-space
// dot(point - planePoint, inwardNormal) <= 0, writing the result into `out`.
int clipPolygon(const Vec3* in, int count, const Vec3& planePoint, const Vec3& inwardNormal, Vec3* out) {
    if (count == 0) {
        return 0;
    }
    int outCount = 0;
    for (int i = 0; i < count; ++i) {
        const Vec3& current = in[i];
        const Vec3& next = in[(i + 1) % count];
        const Real currentDistance = glm::dot(current - planePoint, inwardNormal);
        const Real nextDistance = glm::dot(next - planePoint, inwardNormal);
        const bool currentInside = currentDistance <= 0.0;
        const bool nextInside = nextDistance <= 0.0;
        if (currentInside) {
            out[outCount++] = current;
        }
        if (currentInside != nextInside) {
            const Real t = currentDistance / (currentDistance - nextDistance);
            out[outCount++] = current + (next - current) * t;
        }
    }
    return outCount;
}

// Generates face contacts for two overlapping OBBs given the reference face
// axis on `ref`. `normal` is the unit axis pointing from `ref` to `inc`.
int faceContacts(const Obb& ref, const Obb& inc, int refAxis, const Vec3& normal, ContactPoint* out, int maxPoints) {
    const Vec3 faceCenter = ref.center + normal * component(ref.halfExtents, refAxis);

    // Incident face: the face of `inc` whose normal is most anti-parallel to the
    // reference normal.
    int incidentAxis = 0;
    Real bestAlignment = std::abs(glm::dot(normal, inc.axis[0]));
    for (int i = 1; i < 3; ++i) {
        const Real alignment = std::abs(glm::dot(normal, inc.axis[i]));
        if (alignment > bestAlignment) {
            bestAlignment = alignment;
            incidentAxis = i;
        }
    }
    const Real alignment = glm::dot(normal, inc.axis[incidentAxis]);
    const Vec3 incidentNormal = inc.axis[incidentAxis] * (alignment > 0.0 ? -1.0 : 1.0);
    const Vec3 incidentCenter = inc.center + incidentNormal * component(inc.halfExtents, incidentAxis);

    const int u = (incidentAxis + 1) % 3;
    const int v = (incidentAxis + 2) % 3;
    const Vec3 uAxis = inc.axis[u];
    const Vec3 vAxis = inc.axis[v];
    const Real uHalf = component(inc.halfExtents, u);
    const Real vHalf = component(inc.halfExtents, v);

    Vec3 polygon[8] = {
        incidentCenter - uAxis * uHalf - vAxis * vHalf,
        incidentCenter + uAxis * uHalf - vAxis * vHalf,
        incidentCenter + uAxis * uHalf + vAxis * vHalf,
        incidentCenter - uAxis * uHalf + vAxis * vHalf,
    };
    int pointCount = 4;

    // Clip against the four side planes of the reference face.
    Vec3 buffer[8];
    const int sideAxes[2] = {(refAxis + 1) % 3, (refAxis + 2) % 3};
    for (const int side : sideAxes) {
        const Vec3 axis = ref.axis[side];
        const Real half = component(ref.halfExtents, side);
        pointCount = clipPolygon(polygon, pointCount, faceCenter + axis * half, axis, buffer);
        std::copy(buffer, buffer + pointCount, polygon);
        pointCount = clipPolygon(polygon, pointCount, faceCenter - axis * half, -axis, buffer);
        std::copy(buffer, buffer + pointCount, polygon);
        if (pointCount == 0) {
            break;
        }
    }

    int written = 0;
    for (int i = 0; i < pointCount && written < maxPoints; ++i) {
        const Real separation = glm::dot(polygon[i] - faceCenter, normal);
        if (separation > 0.0) {
            continue;
        }
        out[written].position = polygon[i];
        out[written].normal = normal;
        out[written].penetration = -separation;
        ++written;
    }

    // Degenerate clip (numerical grazing contact): fall back to the deepest
    // support point so the pair still reports a contact.
    if (written == 0) {
        const Vec3 deepest = supportPoint(inc, -normal);
        const Real separation = glm::dot(deepest - faceCenter, normal);
        out[0].position = deepest;
        out[0].normal = normal;
        out[0].penetration = std::max(0.0, -separation);
        written = 1;
    }
    return written;
}

void closestPointsOnSegments(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, Vec3& c1, Vec3& c2) {
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const Real a = glm::dot(d1, d1);
    const Real e = glm::dot(d2, d2);
    const Real f = glm::dot(d2, r);
    Real s = 0.0;
    Real t = 0.0;
    if (a <= 1e-24 && e <= 1e-24) {
        c1 = p1;
        c2 = p2;
        return;
    }
    if (a <= 1e-24) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const Real c = glm::dot(d1, r);
        if (e <= 1e-24) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const Real b = glm::dot(d1, d2);
            const Real denominator = a * e - b * b;
            s = denominator > 1e-24 ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

int edgeContact(const Obb& a, const Obb& b, int axisA, int axisB, const Vec3& normal, ContactPoint* out) {
    const int otherA[2] = {(axisA + 1) % 3, (axisA + 2) % 3};
    const int otherB[2] = {(axisB + 1) % 3, (axisB + 2) % 3};

    Vec3 p1 = a.center;
    Vec3 p2 = b.center;
    for (int i = 0; i < 2; ++i) {
        const Vec3 axis = a.axis[otherA[i]];
        const Real sign = glm::dot(b.center - a.center, axis) >= 0.0 ? 1.0 : -1.0;
        p1 += axis * (sign * component(a.halfExtents, otherA[i]));
    }
    for (int i = 0; i < 2; ++i) {
        const Vec3 axis = b.axis[otherB[i]];
        const Real sign = glm::dot(a.center - b.center, axis) >= 0.0 ? 1.0 : -1.0;
        p2 += axis * (sign * component(b.halfExtents, otherB[i]));
    }
    const Vec3 q1 = p1 + a.axis[axisA] * component(a.halfExtents, axisA);
    const Vec3 r1 = p1 - a.axis[axisA] * component(a.halfExtents, axisA);
    const Vec3 q2 = p2 + b.axis[axisB] * component(b.halfExtents, axisB);
    const Vec3 r2 = p2 - b.axis[axisB] * component(b.halfExtents, axisB);

    Vec3 c1;
    Vec3 c2;
    closestPointsOnSegments(q1, r1, q2, r2, c1, c2);

    const Real projectedA = component(a.halfExtents, 0) * std::abs(glm::dot(normal, a.axis[0])) +
                            component(a.halfExtents, 1) * std::abs(glm::dot(normal, a.axis[1])) +
                            component(a.halfExtents, 2) * std::abs(glm::dot(normal, a.axis[2]));
    const Real projectedB = component(b.halfExtents, 0) * std::abs(glm::dot(normal, b.axis[0])) +
                            component(b.halfExtents, 1) * std::abs(glm::dot(normal, b.axis[1])) +
                            component(b.halfExtents, 2) * std::abs(glm::dot(normal, b.axis[2]));
    const Real separation = glm::dot(b.center - a.center, normal);
    const Real depth = projectedA + projectedB - separation;

    out[0].normal = normal;
    out[0].position = 0.5 * (c1 + c2);
    out[0].penetration = std::max(0.0, depth);
    return 1;
}

int boxBox(const RigidBody& a, const RigidBody& b, ContactPoint* out, int maxPoints, Real margin) {
    const Obb obbA = makeObb(a);
    const Obb obbB = makeObb(b);
    const Vec3 delta = obbB.center - obbA.center;

    Real rotation[3][3];
    Real absolute[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation[i][j] = glm::dot(obbA.axis[i], obbB.axis[j]);
            absolute[i][j] = std::abs(rotation[i][j]) + 1e-12;
        }
    }

    Real minimumOverlap = kInfinity;
    int feature = -1; // 0..2: face of A, 3..5: face of B, 6..14: edge pair
    int featureA = 0;
    int featureB = 0;

    // Separating axis closest to contact (the negative overlap nearest zero),
    // used only when the boxes do not overlap, to build speculative contacts.
    Real closestSeparation = -kInfinity;
    int separationFeature = -1;
    int separationA = 0;
    int separationB = 0;

    auto consider = [&](Real overlap, int tag, int axisA, int axisB) {
        if (overlap < minimumOverlap - kOverlapTie) {
            minimumOverlap = overlap;
            feature = tag;
            featureA = axisA;
            featureB = axisB;
        }
        if (overlap <= 0.0 && overlap > closestSeparation + kOverlapTie) {
            closestSeparation = overlap;
            separationFeature = tag;
            separationA = axisA;
            separationB = axisB;
        }
    };

    // Faces of A.
    for (int i = 0; i < 3; ++i) {
        const Real projectedB = component(obbB.halfExtents, 0) * absolute[i][0] +
                                component(obbB.halfExtents, 1) * absolute[i][1] +
                                component(obbB.halfExtents, 2) * absolute[i][2];
        const Real distance = std::abs(glm::dot(delta, obbA.axis[i]));
        const Real overlap = component(obbA.halfExtents, i) + projectedB - distance;
        consider(overlap, i, 0, 0);
    }

    // Faces of B.
    for (int j = 0; j < 3; ++j) {
        const Real projectedA = component(obbA.halfExtents, 0) * absolute[0][j] +
                                component(obbA.halfExtents, 1) * absolute[1][j] +
                                component(obbA.halfExtents, 2) * absolute[2][j];
        const Real distance = std::abs(glm::dot(delta, obbB.axis[j]));
        const Real overlap = projectedA + component(obbB.halfExtents, j) - distance;
        consider(overlap, 3 + j, 0, 0);
    }

    // Edge-edge axes.
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Vec3 axis = glm::cross(obbA.axis[i], obbB.axis[j]);
            const Real length = glm::length(axis);
            if (length < 1e-9) {
                continue;
            }
            const Vec3 unit = axis / length;
            Real projectedA = 0.0;
            Real projectedB = 0.0;
            for (int k = 0; k < 3; ++k) {
                projectedA += component(obbA.halfExtents, k) * std::abs(glm::dot(unit, obbA.axis[k]));
                projectedB += component(obbB.halfExtents, k) * std::abs(glm::dot(unit, obbB.axis[k]));
            }
            const Real distance = std::abs(glm::dot(delta, unit));
            const Real overlap = projectedA + projectedB - distance;
            consider(overlap, 6 + i * 3 + j, i, j);
        }
    }

    if (minimumOverlap > 0.0) {
        if (feature < 3) {
            const Vec3 normal = obbA.axis[feature] * (glm::dot(delta, obbA.axis[feature]) >= 0.0 ? 1.0 : -1.0);
            return faceContacts(obbA, obbB, feature, normal, out, maxPoints);
        }
        if (feature < 6) {
            // Reference is B: the face normal must point from B towards A (the
            // incident box); flip the reported normals so the manifold still
            // points from A to B.
            const int face = feature - 3;
            const Vec3 refNormal = obbB.axis[face] * (glm::dot(-delta, obbB.axis[face]) >= 0.0 ? 1.0 : -1.0);
            const int count = faceContacts(obbB, obbA, face, refNormal, out, maxPoints);
            for (int i = 0; i < count; ++i) {
                out[i].normal = -out[i].normal;
            }
            return count;
        }

        const Vec3 axis = glm::normalize(glm::cross(obbA.axis[featureA], obbB.axis[featureB]));
        const Vec3 normal = axis * (glm::dot(delta, axis) >= 0.0 ? 1.0 : -1.0);
        return edgeContact(obbA, obbB, featureA, featureB, normal, out);
    }

    // Separated: only report a speculative contact (negative penetration) when
    // the closest separating axis stays within the search margin.
    if (separationFeature < 0 || closestSeparation >= 0.0 || closestSeparation < -margin) {
        return 0;
    }
    Vec3 axis;
    if (separationFeature < 3) {
        axis = obbA.axis[separationFeature];
    } else if (separationFeature < 6) {
        axis = obbB.axis[separationFeature - 3];
    } else {
        axis = glm::normalize(glm::cross(obbA.axis[separationA], obbB.axis[separationB]));
    }
    const Vec3 normal = axis * (glm::dot(delta, axis) >= 0.0 ? 1.0 : -1.0);
    out[0].normal = normal;
    out[0].position = 0.5 * (supportPoint(obbA, normal) + supportPoint(obbB, -normal));
    out[0].penetration = closestSeparation;
    return 1;
}

// Closest point of a triangle (a, b, c) to p (Ericson, Real-Time Collision
// Detection).
Vec3 closestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = p - a;
    const Real d1 = glm::dot(ab, ap);
    const Real d2 = glm::dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return a;
    }
    const Vec3 bp = p - b;
    const Real d3 = glm::dot(ab, bp);
    const Real d4 = glm::dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return b;
    }
    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        return a + ab * (d1 / (d1 - d3));
    }
    const Vec3 cp = p - c;
    const Real d5 = glm::dot(ab, cp);
    const Real d6 = glm::dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return c;
    }
    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        return a + ac * (d2 / (d2 - d6));
    }
    const Real va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    const Real denom = 1.0 / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

// The two triangles of a heightfield cell, in world space.
void cellTriangles(const Heightfield& field, int ix, int iz, Vec3 tri[2][3]) {
    const Real x0 = field.origin.x + static_cast<Real>(ix) * field.spacing;
    const Real x1 = x0 + field.spacing;
    const Real z0 = field.origin.z + static_cast<Real>(iz) * field.spacing;
    const Real z1 = z0 + field.spacing;
    const Vec3 p00(x0, field.heightAt(ix, iz), z0);
    const Vec3 p10(x1, field.heightAt(ix + 1, iz), z0);
    const Vec3 p01(x0, field.heightAt(ix, iz + 1), z1);
    const Vec3 p11(x1, field.heightAt(ix + 1, iz + 1), z1);
    tri[0][0] = p00;
    tri[0][1] = p10;
    tri[0][2] = p11;
    tri[1][0] = p00;
    tri[1][1] = p11;
    tri[1][2] = p01;
}

int cellIndex(Real value, Real origin, Real spacing) {
    return static_cast<int>(std::floor((value - origin) / spacing));
}

// Sphere (b) against terrain (a); the normal points from the terrain to the
// sphere.
int sphereHeightfield(const RigidBody& a, const RigidBody& b, ContactPoint* out, Real margin) {
    const Heightfield* field = a.shape.heightfield.get();
    if (field == nullptr || !field->valid()) {
        return 0;
    }
    const Vec3 center = b.position;
    const Real radius = b.shape.radius;
    const int ix0 = std::max(0, cellIndex(center.x - radius, field->origin.x, field->spacing));
    const int ix1 = std::min(field->width - 2, cellIndex(center.x + radius, field->origin.x, field->spacing));
    const int iz0 = std::max(0, cellIndex(center.z - radius, field->origin.z, field->spacing));
    const int iz1 = std::min(field->depth - 2, cellIndex(center.z + radius, field->origin.z, field->spacing));
    if (ix1 < ix0 || iz1 < iz0) {
        return 0;
    }

    Real bestPenetration = -kInfinity;
    Vec3 bestNormal(0.0, 1.0, 0.0);
    Vec3 bestPoint(center);
    for (int iz = iz0; iz <= iz1; ++iz) {
        for (int ix = ix0; ix <= ix1; ++ix) {
            Vec3 tri[2][3];
            cellTriangles(*field, ix, iz, tri);
            for (int t = 0; t < 2; ++t) {
                const Vec3 q = closestPointOnTriangle(center, tri[t][0], tri[t][1], tri[t][2]);
                const Vec3 delta = center - q;
                const Real distance = glm::length(delta);
                if (distance < radius + margin && radius - distance > bestPenetration) {
                    bestPenetration = radius - distance;
                    bestNormal = distance > 1e-12 ? delta / distance : Vec3(0.0, 1.0, 0.0);
                    bestPoint = q;
                }
            }
        }
    }
    // Deep embedding can hide the sphere centre under the surface; the vertical
    // sample recovers it.
    const Real surface = field->sampleHeight(center.x, center.z);
    const Real vertical = (surface + radius) - center.y;
    if (vertical > bestPenetration) {
        bestPenetration = vertical;
        bestNormal = field->sampleNormal(center.x, center.z);
        bestPoint = Vec3(center.x, surface, center.z);
    }
    if (bestPenetration < -margin) {
        return 0;
    }
    out[0].normal = bestNormal;
    out[0].penetration = bestPenetration;
    out[0].position = bestPoint;
    return 1;
}

// Box (b) against terrain (a). Each corner below the surface becomes a contact;
// the normal points from the terrain to the box. The four deepest points are
// kept (deterministic), which is exactly a box's support face when it rests
// flat.
int boxHeightfield(const RigidBody& a, const RigidBody& b, ContactPoint* out, int maxPoints, Real margin) {
    const Heightfield* field = a.shape.heightfield.get();
    if (field == nullptr || !field->valid()) {
        return 0;
    }
    const Vec3 he = b.shape.halfExtents;
    struct Candidate {
        Real penetration;
        Vec3 normal;
        Vec3 point;
        int corner;
    };
    std::vector<Candidate> candidates;
    int corner = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                const Vec3 world = b.localToWorldPoint(Vec3(sx * he.x, sy * he.y, sz * he.z));
                const int index = corner++;
                if (!field->contains(world.x, world.z)) {
                    continue;
                }
                const Vec3 surfacePoint(world.x, field->sampleHeight(world.x, world.z), world.z);
                const Vec3 normal = field->sampleNormal(world.x, world.z);
                const Real penetration = glm::dot(surfacePoint - world, normal);
                if (penetration > -margin) {
                    candidates.push_back(Candidate{penetration, normal, world, index});
                }
            }
        }
    }
    if (candidates.empty()) {
        return 0;
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& l, const Candidate& r) {
        if (l.penetration != r.penetration) {
            return l.penetration > r.penetration;
        }
        return l.corner < r.corner;
    });
    const int count = std::min<int>(maxPoints, static_cast<int>(candidates.size()));
    // Emit in corner order so warm-start rows keep matching across frames.
    std::sort(candidates.begin(), candidates.begin() + count,
              [](const Candidate& l, const Candidate& r) { return l.corner < r.corner; });
    for (int i = 0; i < count; ++i) {
        out[i].normal = candidates[i].normal;
        out[i].penetration = candidates[i].penetration;
        out[i].position = candidates[i].point;
    }
    return count;
}

// Conservative radius of a shape about its centre, used to turn an angular
// speed into the fastest linear speed of any of its points.
Real boundingRadius(const RigidBody& body) {
    const Shape& shape = body.shape;
    switch (shape.type) {
    case ShapeType::Sphere:
        return shape.radius;
    case ShapeType::Cylinder:
        return std::sqrt(shape.radius * shape.radius + shape.halfHeight * shape.halfHeight);
    case ShapeType::Box:
        return glm::length(shape.halfExtents);
    case ShapeType::Convex: {
        Real radius = 0.0;
        for (const Vec3& vertex : shape.vertices) {
            radius = std::max(radius, glm::length(vertex));
        }
        return radius;
    }
    case ShapeType::Heightfield:
        return 0.0;
    }
    return 0.0;
}

} // namespace

Aabb computeAabb(const RigidBody& body) {
    switch (body.shape.type) {
    case ShapeType::Sphere: {
        const Vec3 radius(body.shape.radius);
        return Aabb{body.position - radius, body.position + radius};
    }
    case ShapeType::Cylinder: {
        const Vec3 radialX = glm::abs(body.localToWorldDirection(Vec3(1.0, 0.0, 0.0)));
        const Vec3 radialZ = glm::abs(body.localToWorldDirection(Vec3(0.0, 0.0, 1.0)));
        const Vec3 axial = glm::abs(body.localToWorldDirection(Vec3(0.0, 1.0, 0.0)));
        const Vec3 extent = body.shape.radius * (radialX + radialZ) + body.shape.halfHeight * axial;
        return Aabb{body.position - extent, body.position + extent};
    }
    case ShapeType::Heightfield: {
        const Heightfield* field = body.shape.heightfield.get();
        if (field == nullptr || !field->valid()) {
            return Aabb{body.position, body.position};
        }
        // The field is authored in world space, so the body transform is ignored.
        return Aabb{field->minCorner(), field->maxCorner()};
    }
    case ShapeType::Box:
    default: {
        const Vec3 axisX = glm::abs(body.localToWorldDirection(Vec3(1.0, 0.0, 0.0)));
        const Vec3 axisY = glm::abs(body.localToWorldDirection(Vec3(0.0, 1.0, 0.0)));
        const Vec3 axisZ = glm::abs(body.localToWorldDirection(Vec3(0.0, 0.0, 1.0)));
        const Vec3 extent =
            body.shape.halfExtents.x * axisX + body.shape.halfExtents.y * axisY + body.shape.halfExtents.z * axisZ;
        return Aabb{body.position - extent, body.position + extent};
    }
    }
}

int collide(const RigidBody& a, const RigidBody& b, ContactPoint* out, int maxPoints, Real margin) {
    if (maxPoints <= 0) {
        return 0;
    }
    const ShapeType typeA = a.shape.type;
    const ShapeType typeB = b.shape.type;

    // Convex hulls (and any future shape without an analytic routine) go through
    // the generic GJK/EPA path, which returns a single contact point. It reports
    // real overlaps only, so `margin` is ignored for these pairs.
    if (typeA == ShapeType::Convex || typeB == ShapeType::Convex) {
        const ConvexContact contact = convexContact(a, b);
        if (!contact.intersecting) {
            return 0;
        }
        out[0].normal = contact.normal;
        out[0].penetration = contact.depth;
        out[0].position = 0.5 * (contact.pointA + contact.pointB);
        return 1;
    }

    if (typeA == ShapeType::Sphere && typeB == ShapeType::Sphere) {
        return sphereSphere(a, b, out, margin);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Box) {
        return sphereBox(a, b, out, margin);
    }
    if (typeA == ShapeType::Box && typeB == ShapeType::Sphere) {
        const int count = sphereBox(b, a, out, margin);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Box && typeB == ShapeType::Box) {
        return boxBox(a, b, out, maxPoints, margin);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Cylinder) {
        return sphereCylinder(a, b, out, margin);
    }
    if (typeA == ShapeType::Cylinder && typeB == ShapeType::Sphere) {
        const int count = sphereCylinder(b, a, out, margin);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Heightfield && typeB == ShapeType::Sphere) {
        return sphereHeightfield(a, b, out, margin);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Heightfield) {
        const int count = sphereHeightfield(b, a, out, margin);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Heightfield && typeB == ShapeType::Box) {
        return boxHeightfield(a, b, out, maxPoints, margin);
    }
    if (typeA == ShapeType::Box && typeB == ShapeType::Heightfield) {
        const int count = boxHeightfield(b, a, out, maxPoints, margin);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    // Box/cylinder combinations without an analytic routine (box-cylinder and
    // cylinder-cylinder) go through the generic GJK/EPA path, which returns a
    // single witness point. Heightfield pairs are excluded: supportWorld only
    // approximates a heightfield as a unit box, so they must not take this route.
    const bool boxOrCylinderA = typeA == ShapeType::Box || typeA == ShapeType::Cylinder;
    const bool boxOrCylinderB = typeB == ShapeType::Box || typeB == ShapeType::Cylinder;
    if (boxOrCylinderA && boxOrCylinderB) {
        const ConvexContact contact = convexContact(a, b);
        if (!contact.intersecting) {
            return 0;
        }
        out[0].normal = contact.normal;
        out[0].penetration = contact.depth;
        out[0].position = 0.5 * (contact.pointA + contact.pointB);
        return 1;
    }
    return 0;
}

std::vector<BodyPair> broadphase(const PhysicsWorld& world, Real velocityMarginDt) {
    const std::size_t count = world.bodyCount();
    PhysicsWorld::BroadphaseCache& cache = world.broadphaseCache_;

    // Static geometry never moves, so its tree is built once and reused: a scene
    // with hundreds of walls and props would otherwise rebuild the whole tree
    // every step. Only the handful of dynamic bodies is re-boxed each call.
    if (!cache.valid || cache.builtBodyCount != count) {
        // `boxes` is indexed by body index (a leaf stores boxes[order_[i]]), so it
        // spans every body even though only the static ones are inserted.
        std::vector<Aabb> staticBoxes(count);
        std::vector<std::size_t> staticIndices;
        staticIndices.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const RigidBody& body = world.body(i);
            if (body.collisionEnabled && body.isStatic()) {
                staticIndices.push_back(i);
                staticBoxes[i] = computeAabb(body);
            }
        }
        cache.tree.build(staticBoxes, staticIndices);
        cache.builtBodyCount = count;
        cache.valid = true;
    }

    std::vector<BodyPair> pairs;
    std::vector<std::size_t> dynamicIndices;
    std::vector<Aabb> dynamicBoxes;
    for (std::size_t i = 0; i < count; ++i) {
        const RigidBody& body = world.body(i);
        if (!body.collisionEnabled || body.isStatic()) {
            continue;
        }
        Aabb box = computeAabb(body);
        if (velocityMarginDt > 0.0) {
            const Real speed =
                glm::length(body.linearVelocity) + glm::length(body.angularVelocity) * boundingRadius(body);
            if (speed > 0.0) {
                const Vec3 inflation(speed * velocityMarginDt);
                box.min -= inflation;
                box.max += inflation;
            }
        }
        dynamicIndices.push_back(i);
        dynamicBoxes.push_back(box);
    }

    // Dynamic vs static: query the cached tree. Dynamic vs dynamic: brute force
    // over the (few) moving bodies. This reproduces the full self-collision pair
    // set, since two static bodies never form a contact anyway.
    std::vector<std::size_t> hits;
    for (std::size_t d = 0; d < dynamicIndices.size(); ++d) {
        hits.clear();
        cache.tree.query(dynamicBoxes[d], hits);
        for (const std::size_t staticIndex : hits) {
            const std::size_t bodyA = std::min(dynamicIndices[d], staticIndex);
            const std::size_t bodyB = std::max(dynamicIndices[d], staticIndex);
            pairs.push_back(BodyPair{bodyA, bodyB});
        }
        for (std::size_t e = d + 1; e < dynamicIndices.size(); ++e) {
            if (!dynamicBoxes[d].overlaps(dynamicBoxes[e])) {
                continue;
            }
            const std::size_t bodyA = std::min(dynamicIndices[d], dynamicIndices[e]);
            const std::size_t bodyB = std::max(dynamicIndices[d], dynamicIndices[e]);
            pairs.push_back(BodyPair{bodyA, bodyB});
        }
    }

    std::sort(pairs.begin(), pairs.end(), [](const BodyPair& lhs, const BodyPair& rhs) {
        if (lhs.bodyA != rhs.bodyA) {
            return lhs.bodyA < rhs.bodyA;
        }
        return lhs.bodyB < rhs.bodyB;
    });
    pairs.erase(std::unique(pairs.begin(), pairs.end(),
                            [](const BodyPair& lhs, const BodyPair& rhs) {
                                return lhs.bodyA == rhs.bodyA && lhs.bodyB == rhs.bodyB;
                            }),
                pairs.end());
    return pairs;
}

std::vector<ContactManifold> detectCollisions(const PhysicsWorld& world, Real margin, Real velocityMarginDt) {
    std::vector<ContactManifold> manifolds;
    for (const BodyPair& pair : broadphase(world, velocityMarginDt)) {
        const RigidBody& bodyA = world.body(pair.bodyA);
        const RigidBody& bodyB = world.body(pair.bodyB);
        Real pairMargin = margin;
        if (velocityMarginDt > 0.0) {
            const Real speedA =
                glm::length(bodyA.linearVelocity) + glm::length(bodyA.angularVelocity) * boundingRadius(bodyA);
            const Real speedB =
                glm::length(bodyB.linearVelocity) + glm::length(bodyB.angularVelocity) * boundingRadius(bodyB);
            pairMargin += (speedA + speedB) * velocityMarginDt;
        }
        ContactManifold manifold;
        manifold.bodyA = pair.bodyA;
        manifold.bodyB = pair.bodyB;
        manifold.count = collide(bodyA, bodyB, manifold.points, kMaxContactPoints, pairMargin);
        if (manifold.count > 0) {
            manifolds.push_back(manifold);
        }
    }
    return manifolds;
}

} // namespace sim::physics
