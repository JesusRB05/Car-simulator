// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Convex.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sim::physics {

Vec3 supportWorld(const RigidBody& body, const Vec3& direction) {
    switch (body.shape.type) {
    case ShapeType::Sphere:
        return body.position + safeNormalize(direction, Vec3(0.0, 1.0, 0.0)) * body.shape.radius;
    case ShapeType::Cylinder: {
        const Vec3 axis = body.localToWorldDirection(Vec3(0.0, 1.0, 0.0));
        const Real axial = (glm::dot(direction, axis) >= 0.0 ? 1.0 : -1.0) * body.shape.halfHeight;
        const Vec3 radial = direction - axis * glm::dot(direction, axis);
        return body.position + axis * axial + safeNormalize(radial, Vec3(0.0)) * body.shape.radius;
    }
    case ShapeType::Convex: {
        if (body.shape.vertices.empty()) {
            return body.position;
        }
        const Vec3 localDirection = rotateVector(quaternionConjugate(body.orientation), direction);
        Real best = -kInfinity;
        Vec3 bestVertex = body.shape.vertices.front();
        for (const Vec3& vertex : body.shape.vertices) {
            const Real projection = glm::dot(vertex, localDirection);
            if (projection > best) {
                best = projection;
                bestVertex = vertex;
            }
        }
        return body.localToWorldPoint(bestVertex);
    }
    case ShapeType::Box:
    case ShapeType::Heightfield:
    default: {
        const Vec3 axis[3] = {body.localToWorldDirection(Vec3(1.0, 0.0, 0.0)),
                              body.localToWorldDirection(Vec3(0.0, 1.0, 0.0)),
                              body.localToWorldDirection(Vec3(0.0, 0.0, 1.0))};
        const Real half[3] = {body.shape.halfExtents.x, body.shape.halfExtents.y, body.shape.halfExtents.z};
        Vec3 point = body.position;
        for (int i = 0; i < 3; ++i) {
            point += axis[i] * ((glm::dot(direction, axis[i]) >= 0.0 ? 1.0 : -1.0) * half[i]);
        }
        return point;
    }
    }
}

namespace {

struct Support {
    Vec3 w;  // point on A minus point on B
    Vec3 pa; // on A (world)
    Vec3 pb; // on B (world)
};

Support minkowskiSupport(const RigidBody& a, const RigidBody& b, const Vec3& direction) {
    const Vec3 pa = supportWorld(a, direction);
    const Vec3 pb = supportWorld(b, -direction);
    return Support{pa - pb, pa, pb};
}

// Simplex with the newest point at index 0.
struct Simplex {
    Vec3 w[4];
    Vec3 pa[4];
    Vec3 pb[4];
    int count = 0;

    void pushFront(const Support& s) {
        for (int i = count; i > 0; --i) {
            w[i] = w[i - 1];
            pa[i] = pa[i - 1];
            pb[i] = pb[i - 1];
        }
        w[0] = s.w;
        pa[0] = s.pa;
        pb[0] = s.pb;
        if (count < 4) {
            ++count;
        }
    }

    void keep(int count, int a, int b = 0, int c = 0) {
        // Reorders to (a, b, c) then truncates to `count`.
        const Vec3 nw[3] = {w[a], w[b], w[c]};
        const Vec3 na[3] = {pa[a], pa[b], pa[c]};
        const Vec3 nb[3] = {pb[a], pb[b], pb[c]};
        for (int i = 0; i < count; ++i) {
            w[i] = nw[i];
            pa[i] = na[i];
            pb[i] = nb[i];
        }
        this->count = count;
    }
};

bool sameDirection(const Vec3& a, const Vec3& b) {
    return glm::dot(a, b) > 0.0;
}

constexpr Real kEpsilon = 1e-14;

// Returns true when the origin is enclosed (the shapes intersect).
bool updateSimplex(Simplex& s, Vec3& d) {
    if (s.count == 2) {
        const Vec3 a = s.w[0];
        const Vec3 b = s.w[1];
        const Vec3 ab = b - a;
        const Vec3 ao = -a;
        if (sameDirection(ab, ao)) {
            d = glm::cross(glm::cross(ab, ao), ab);
            if (glm::dot(d, d) < kEpsilon) {
                return true;
            }
        } else {
            s.count = 1;
            d = ao;
        }
        return false;
    }
    if (s.count == 3) {
        const Vec3 a = s.w[0];
        const Vec3 b = s.w[1];
        const Vec3 c = s.w[2];
        const Vec3 ab = b - a;
        const Vec3 ac = c - a;
        const Vec3 ao = -a;
        const Vec3 abc = glm::cross(ab, ac);
        if (sameDirection(glm::cross(abc, ac), ao)) {
            if (sameDirection(ac, ao)) {
                s.keep(2, 0, 2);
                d = glm::cross(glm::cross(ac, ao), ac);
                if (glm::dot(d, d) < kEpsilon) {
                    return true;
                }
            } else {
                if (sameDirection(ab, ao)) {
                    s.keep(2, 0, 1);
                    d = glm::cross(glm::cross(ab, ao), ab);
                    if (glm::dot(d, d) < kEpsilon) {
                        return true;
                    }
                } else {
                    s.keep(1, 0);
                    d = ao;
                }
            }
        } else if (sameDirection(glm::cross(ab, abc), ao)) {
            if (sameDirection(ab, ao)) {
                s.keep(2, 0, 1);
                d = glm::cross(glm::cross(ab, ao), ab);
                if (glm::dot(d, d) < kEpsilon) {
                    return true;
                }
            } else {
                s.keep(1, 0);
                d = ao;
            }
        } else {
            if (sameDirection(abc, ao)) {
                d = abc;
            } else {
                s.keep(3, 0, 2, 1);
                d = -abc;
            }
        }
        return false;
    }
    // count == 4
    const Vec3 a = s.w[0];
    const Vec3 ab = s.w[1] - a;
    const Vec3 ac = s.w[2] - a;
    const Vec3 ad = s.w[3] - a;
    const Vec3 ao = -a;
    const Vec3 abc = glm::cross(ab, ac);
    const Vec3 acd = glm::cross(ac, ad);
    const Vec3 adb = glm::cross(ad, ab);
    if (sameDirection(abc, ao)) {
        s.keep(3, 0, 1, 2);
        return updateSimplex(s, d);
    }
    if (sameDirection(acd, ao)) {
        s.keep(3, 0, 2, 3);
        return updateSimplex(s, d);
    }
    if (sameDirection(adb, ao)) {
        s.keep(3, 0, 3, 1);
        return updateSimplex(s, d);
    }
    return true;
}

bool gjk(const RigidBody& a, const RigidBody& b, Simplex& simplex) {
    // Seed with a deterministic, slightly off-axis direction: a purely radial
    // seed can put the first two support points exactly collinear with the
    // origin (two spheres on a coordinate axis), which would hand EPA a
    // lower-dimensional simplex it cannot expand.
    Vec3 direction = a.position - b.position + Vec3(1.0, 1.0, 1.0) * 1e-3;
    if (glm::dot(direction, direction) < kEpsilon) {
        direction = Vec3(0.0, 1.0, 0.0);
    }
    simplex.count = 0;
    simplex.pushFront(minkowskiSupport(a, b, direction));
    direction = -simplex.w[0];
    for (int iteration = 0; iteration < 64; ++iteration) {
        if (glm::dot(direction, direction) < kEpsilon) {
            return true;
        }
        const Support next = minkowskiSupport(a, b, direction);
        if (!sameDirection(next.w, direction)) {
            return false;
        }
        simplex.pushFront(next);
        if (updateSimplex(simplex, direction)) {
            return true;
        }
    }
    return true;
}

struct EpaFace {
    int i = 0;
    int j = 0;
    int k = 0;
    Vec3 normal{0.0, 1.0, 0.0};
    Real distance = 0.0;
};

bool addFace(const std::vector<Vec3>& w, std::vector<EpaFace>& faces, int i, int j, int k) {
    Vec3 normal = glm::cross(w[j] - w[i], w[k] - w[i]);
    const Real length = glm::length(normal);
    if (length < 1e-12) {
        return false;
    }
    normal /= length;
    Real distance = glm::dot(normal, w[i]);
    if (distance < 0.0) {
        normal = -normal;
        distance = -distance;
        std::swap(j, k);
    }
    faces.push_back(EpaFace{i, j, k, normal, distance});
    return true;
}

void addEdge(std::vector<std::pair<int, int>>& horizon, int a, int b) {
    for (std::size_t i = 0; i < horizon.size(); ++i) {
        if (horizon[i].first == b && horizon[i].second == a) {
            horizon.erase(horizon.begin() + static_cast<std::ptrdiff_t>(i));
            return;
        }
    }
    horizon.emplace_back(a, b);
}

// Expands the enclosing GJK tetrahedron to the closest face of the Minkowski
// difference, returning the penetration depth, normal and witness points.
ConvexContact epa(const RigidBody& a, const RigidBody& b, const Simplex& simplex) {
    std::vector<Vec3> w(simplex.w, simplex.w + simplex.count);
    std::vector<Vec3> pa(simplex.pa, simplex.pa + simplex.count);
    std::vector<Vec3> pb(simplex.pb, simplex.pb + simplex.count);
    if (w.size() < 4) {
        return ConvexContact{};
    }
    std::vector<EpaFace> faces;
    faces.reserve(32);
    addFace(w, faces, 0, 1, 2);
    addFace(w, faces, 0, 2, 3);
    addFace(w, faces, 0, 3, 1);
    addFace(w, faces, 1, 3, 2);

    for (int iteration = 0; iteration < 64 && !faces.empty(); ++iteration) {
        std::size_t best = 0;
        for (std::size_t i = 1; i < faces.size(); ++i) {
            if (faces[i].distance < faces[best].distance) {
                best = i;
            }
        }
        const EpaFace face = faces[best];
        const Support support = minkowskiSupport(a, b, face.normal);
        const Real reach = glm::dot(support.w, face.normal);
        if (reach - face.distance < 1e-9) {
            // Converged: project the origin onto the face for the witness points.
            const Vec3 p = face.normal * face.distance;
            const Vec3 v0 = w[face.i];
            const Vec3 a0 = w[face.j] - v0;
            const Vec3 a1 = w[face.k] - v0;
            const Vec3 ap = p - v0;
            const Real d00 = glm::dot(a0, a0);
            const Real d01 = glm::dot(a0, a1);
            const Real d11 = glm::dot(a1, a1);
            const Real d20 = glm::dot(ap, a0);
            const Real d21 = glm::dot(ap, a1);
            const Real denom = d00 * d11 - d01 * d01;
            Real v = 0.0;
            Real u = 0.0;
            if (std::abs(denom) > 1e-18) {
                v = (d11 * d20 - d01 * d21) / denom;
                u = (d00 * d21 - d01 * d20) / denom;
            }
            v = std::clamp(v, 0.0, 1.0);
            u = std::clamp(u, 0.0, 1.0);
            const Real t = std::clamp(1.0 - v - u, 0.0, 1.0);
            ConvexContact result;
            result.intersecting = true;
            result.normal = face.normal;
            result.depth = face.distance;
            result.pointA = t * pa[face.i] + v * pa[face.j] + u * pa[face.k];
            result.pointB = t * pb[face.i] + v * pb[face.j] + u * pb[face.k];
            return result;
        }

        const int index = static_cast<int>(w.size());
        w.push_back(support.w);
        pa.push_back(support.pa);
        pb.push_back(support.pb);

        std::vector<std::pair<int, int>> horizon;
        std::vector<EpaFace> kept;
        kept.reserve(faces.size());
        for (const EpaFace& f : faces) {
            if (glm::dot(f.normal, support.w - w[f.i]) > 1e-10) {
                addEdge(horizon, f.i, f.j);
                addEdge(horizon, f.j, f.k);
                addEdge(horizon, f.k, f.i);
            } else {
                kept.push_back(f);
            }
        }
        faces.swap(kept);
        for (const std::pair<int, int>& edge : horizon) {
            addFace(w, faces, edge.first, edge.second, index);
        }
    }

    // Fallback if EPA fails to converge: report the shallowest known face.
    if (!faces.empty()) {
        const EpaFace& face = *std::min_element(
            faces.begin(), faces.end(), [](const EpaFace& l, const EpaFace& r) { return l.distance < r.distance; });
        ConvexContact result;
        result.intersecting = true;
        result.normal = face.normal;
        result.depth = face.distance;
        result.pointA = pa[face.i];
        result.pointB = pb[face.i];
        return result;
    }
    return ConvexContact{};
}

} // namespace

bool convexOverlap(const RigidBody& a, const RigidBody& b) {
    Simplex simplex;
    return gjk(a, b, simplex);
}

ConvexContact convexContact(const RigidBody& a, const RigidBody& b) {
    Simplex simplex;
    if (!gjk(a, b, simplex)) {
        return ConvexContact{};
    }
    if (simplex.count < 4) {
        // Touching (or lower-dimensional contact): no meaningful depth.
        ConvexContact result;
        result.intersecting = true;
        result.normal = safeNormalize(a.position - b.position, Vec3(0.0, 1.0, 0.0));
        result.depth = 0.0;
        result.pointA = supportWorld(a, result.normal);
        result.pointB = supportWorld(b, -result.normal);
        return result;
    }
    return epa(a, b, simplex);
}

} // namespace sim::physics
