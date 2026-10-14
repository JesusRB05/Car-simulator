// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Collision.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "physics/Convex.hpp"
#include "physics/Heightfield.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/TriangleMesh.hpp"

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

int sphereSphere(const RigidBody& a, const RigidBody& b, ContactPoint* out) {
    const Vec3 delta = b.position - a.position;
    const Real distanceSquared = glm::dot(delta, delta);
    const Real sumRadii = a.shape.radius + b.shape.radius;
    if (distanceSquared > sumRadii * sumRadii) {
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
int sphereBox(const RigidBody& a, const RigidBody& b, ContactPoint* out) {
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
        if (distance > a.shape.radius) {
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

int sphereCylinder(const RigidBody& a, const RigidBody& b, ContactPoint* out) {
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
        if (distance > radius) {
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

int boxBox(const RigidBody& a, const RigidBody& b, ContactPoint* out, int maxPoints) {
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

    // Faces of A.
    for (int i = 0; i < 3; ++i) {
        const Real projectedB = component(obbB.halfExtents, 0) * absolute[i][0] +
                                component(obbB.halfExtents, 1) * absolute[i][1] +
                                component(obbB.halfExtents, 2) * absolute[i][2];
        const Real distance = std::abs(glm::dot(delta, obbA.axis[i]));
        const Real overlap = component(obbA.halfExtents, i) + projectedB - distance;
        if (overlap <= 0.0) {
            return 0;
        }
        if (overlap < minimumOverlap - kOverlapTie) {
            minimumOverlap = overlap;
            feature = i;
        }
    }

    // Faces of B.
    for (int j = 0; j < 3; ++j) {
        const Real projectedA = component(obbA.halfExtents, 0) * absolute[0][j] +
                                component(obbA.halfExtents, 1) * absolute[1][j] +
                                component(obbA.halfExtents, 2) * absolute[2][j];
        const Real distance = std::abs(glm::dot(delta, obbB.axis[j]));
        const Real overlap = projectedA + component(obbB.halfExtents, j) - distance;
        if (overlap <= 0.0) {
            return 0;
        }
        if (overlap < minimumOverlap - kOverlapTie) {
            minimumOverlap = overlap;
            feature = 3 + j;
        }
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
            if (overlap <= 0.0) {
                return 0;
            }
            if (overlap < minimumOverlap - kOverlapTie) {
                minimumOverlap = overlap;
                feature = 6 + i * 3 + j;
                featureA = i;
                featureB = j;
            }
        }
    }

    if (feature < 0) {
        return 0;
    }

    if (feature < 3) {
        Vec3 normal = obbA.axis[feature] * (glm::dot(delta, obbA.axis[feature]) >= 0.0 ? 1.0 : -1.0);
        return faceContacts(obbA, obbB, feature, normal, out, maxPoints);
    }
    if (feature < 6) {
        // Reference is B: the face normal must point from B towards A (the
        // incident box); flip the reported normals so the manifold still points
        // from A to B.
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
int sphereHeightfield(const RigidBody& a, const RigidBody& b, ContactPoint* out) {
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

    Real bestPenetration = -1.0;
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
                if (distance < radius && radius - distance > bestPenetration) {
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
    if (bestPenetration < 0.0) {
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
int boxHeightfield(const RigidBody& a, const RigidBody& b, ContactPoint* out, int maxPoints) {
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
                if (penetration > 0.0) {
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

// Sphere (b) against a triangle-mesh body (a); the normal points from the mesh
// to the sphere.
int sphereMesh(const RigidBody& meshBody, const RigidBody& sphere, ContactPoint* out) {
    const TriangleMesh* mesh = meshBody.shape.mesh.get();
    if (mesh == nullptr || !mesh->valid()) {
        return 0;
    }
    const Vec3 center = sphere.position;
    const Real radius = sphere.shape.radius;
    Real bestPenetration = -1.0;
    Vec3 bestNormal(0.0, 1.0, 0.0);
    Vec3 bestPoint(center);
    for (std::size_t t = 0; t < mesh->triangleCount(); ++t) {
        Vec3 ta;
        Vec3 tb;
        Vec3 tc;
        mesh->triangle(t, ta, tb, tc);
        const Vec3 a = meshBody.localToWorldPoint(ta);
        const Vec3 b = meshBody.localToWorldPoint(tb);
        const Vec3 c = meshBody.localToWorldPoint(tc);
        const Vec3 q = closestPointOnTriangle(center, a, b, c);
        const Vec3 delta = center - q;
        const Real distance = glm::length(delta);
        if (distance < radius && radius - distance > bestPenetration) {
            bestPenetration = radius - distance;
            bestNormal = distance > 1e-12 ? delta / distance : Vec3(0.0, 1.0, 0.0);
            bestPoint = q;
        }
    }
    if (bestPenetration < 0.0) {
        return 0;
    }
    out[0].normal = bestNormal;
    out[0].penetration = bestPenetration;
    out[0].position = bestPoint;
    return 1;
}

// Box (b) against a triangle mesh (a). Each corner's nearest triangle, when the
// corner is behind its outward normal, becomes a contact; the four deepest are
// kept (as for the heightfield).
int boxMesh(const RigidBody& meshBody, const RigidBody& box, ContactPoint* out, int maxPoints) {
    const TriangleMesh* mesh = meshBody.shape.mesh.get();
    if (mesh == nullptr || !mesh->valid()) {
        return 0;
    }
    const Vec3 he = box.shape.halfExtents;
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
                const Vec3 world = box.localToWorldPoint(Vec3(sx * he.x, sy * he.y, sz * he.z));
                const int index = corner++;
                Real bestDistanceSquared = kInfinity;
                Vec3 bestPoint;
                Vec3 bestNormal(0.0, 1.0, 0.0);
                for (std::size_t t = 0; t < mesh->triangleCount(); ++t) {
                    Vec3 ta;
                    Vec3 tb;
                    Vec3 tc;
                    mesh->triangle(t, ta, tb, tc);
                    const Vec3 a = meshBody.localToWorldPoint(ta);
                    const Vec3 b = meshBody.localToWorldPoint(tb);
                    const Vec3 c = meshBody.localToWorldPoint(tc);
                    const Vec3 q = closestPointOnTriangle(world, a, b, c);
                    const Real distanceSquared = glm::dot(world - q, world - q);
                    if (distanceSquared < bestDistanceSquared) {
                        bestDistanceSquared = distanceSquared;
                        bestPoint = q;
                        bestNormal = safeNormalize(glm::cross(b - a, c - a), Vec3(0.0, 1.0, 0.0));
                    }
                }
                if (bestDistanceSquared == kInfinity) {
                    continue;
                }
                const Real penetration = glm::dot(bestPoint - world, bestNormal);
                if (penetration > 0.0) {
                    candidates.push_back(Candidate{penetration, bestNormal, world, index});
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
    std::sort(candidates.begin(), candidates.begin() + count,
              [](const Candidate& l, const Candidate& r) { return l.corner < r.corner; });
    for (int i = 0; i < count; ++i) {
        out[i].normal = candidates[i].normal;
        out[i].penetration = candidates[i].penetration;
        out[i].position = candidates[i].point;
    }
    return count;
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
    case ShapeType::Mesh: {
        const TriangleMesh* mesh = body.shape.mesh.get();
        if (mesh == nullptr || mesh->vertices.empty()) {
            return Aabb{body.position, body.position};
        }
        Vec3 min = body.localToWorldPoint(mesh->vertices.front());
        Vec3 max = min;
        for (const Vec3& v : mesh->vertices) {
            const Vec3 world = body.localToWorldPoint(v);
            min = glm::min(min, world);
            max = glm::max(max, world);
        }
        return Aabb{min, max};
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

int collide(const RigidBody& a, const RigidBody& b, ContactPoint* out, int maxPoints) {
    if (maxPoints <= 0) {
        return 0;
    }
    const ShapeType typeA = a.shape.type;
    const ShapeType typeB = b.shape.type;

    // Convex hulls (and any future shape without an analytic routine) go through
    // the generic GJK/EPA path, which returns a single contact point.
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
        return sphereSphere(a, b, out);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Box) {
        return sphereBox(a, b, out);
    }
    if (typeA == ShapeType::Box && typeB == ShapeType::Sphere) {
        const int count = sphereBox(b, a, out);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Box && typeB == ShapeType::Box) {
        return boxBox(a, b, out, maxPoints);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Cylinder) {
        return sphereCylinder(a, b, out);
    }
    if (typeA == ShapeType::Cylinder && typeB == ShapeType::Sphere) {
        const int count = sphereCylinder(b, a, out);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Heightfield && typeB == ShapeType::Sphere) {
        return sphereHeightfield(a, b, out);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Heightfield) {
        const int count = sphereHeightfield(b, a, out);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Heightfield && typeB == ShapeType::Box) {
        return boxHeightfield(a, b, out, maxPoints);
    }
    if (typeA == ShapeType::Box && typeB == ShapeType::Heightfield) {
        const int count = boxHeightfield(b, a, out, maxPoints);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Mesh && typeB == ShapeType::Sphere) {
        return sphereMesh(a, b, out);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Mesh) {
        const int count = sphereMesh(b, a, out);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    if (typeA == ShapeType::Mesh && typeB == ShapeType::Box) {
        return boxMesh(a, b, out, maxPoints);
    }
    if (typeA == ShapeType::Box && typeB == ShapeType::Mesh) {
        const int count = boxMesh(b, a, out, maxPoints);
        for (int i = 0; i < count; ++i) {
            out[i].normal = -out[i].normal;
        }
        return count;
    }
    // Cylinder-cylinder and box-cylinder narrowphase is not implemented yet; the
    // pair is skipped (see docs/ROADMAP.md, Phase 1).
    return 0;
}

std::vector<BodyPair> broadphase(const PhysicsWorld& world) {
    const std::size_t count = world.bodyCount();
    std::vector<Aabb> boxes(count);
    for (std::size_t i = 0; i < count; ++i) {
        boxes[i] = computeAabb(world.body(i));
    }

    std::vector<BodyPair> pairs;
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = i + 1; j < count; ++j) {
            if (!world.body(i).collisionEnabled || !world.body(j).collisionEnabled) {
                continue;
            }
            if (!world.body(i).isDynamic() && !world.body(j).isDynamic()) {
                continue;
            }
            if (boxes[i].overlaps(boxes[j])) {
                pairs.push_back(BodyPair{i, j});
            }
        }
    }
    return pairs;
}

std::vector<ContactManifold> detectCollisions(const PhysicsWorld& world) {
    std::vector<ContactManifold> manifolds;
    for (const BodyPair& pair : broadphase(world)) {
        ContactManifold manifold;
        manifold.bodyA = pair.bodyA;
        manifold.bodyB = pair.bodyB;
        manifold.count = collide(world.body(pair.bodyA), world.body(pair.bodyB), manifold.points, kMaxContactPoints);
        if (manifold.count > 0) {
            manifolds.push_back(manifold);
        }
    }
    return manifolds;
}

} // namespace sim::physics
