// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Frustum.hpp"

namespace sim::render {
namespace {

// Coefficients of one clip-space component: row(r) is the 4-vector whose dot
// product with a point gives clip component r. In a column-major glm::mat4 that
// is (m[0][r], m[1][r], m[2][r], m[3][r]).
glm::vec4 clipRow(int r, const glm::mat4& m) {
    return glm::vec4(m[0][r], m[1][r], m[2][r], m[3][r]);
}

// Unit normal, so distance() is a real distance and the radius test is a plain
// comparison. A degenerate plane (a zero row) cannot reject anything, so it is
// left as is rather than dividing by zero.
void normalize(glm::vec4& plane) {
    const float length = glm::length(glm::vec3(plane));
    if (length > 0.0f) {
        plane /= length;
    }
}

float dot(const glm::vec4& plane, const glm::vec3& point) {
    return plane.x * point.x + plane.y * point.y + plane.z * point.z + plane.w;
}

} // namespace

Aabb transformBounds(const Aabb& local, const glm::mat4& transform) {
    Aabb result;
    for (int corner = 0; corner < 8; ++corner) {
        const glm::vec3 p(corner & 1 ? local.hi.x : local.lo.x, corner & 2 ? local.hi.y : local.lo.y,
                          corner & 4 ? local.hi.z : local.lo.z);
        const glm::vec3 mapped = glm::vec3(transform * glm::vec4(p, 1.0f));
        // Grow from the first mapped corner: starting from the local box would
        // keep parts of it whenever the transform only translates.
        if (corner == 0) {
            result = Aabb{mapped, mapped};
            continue;
        }
        result.lo = glm::min(result.lo, mapped);
        result.hi = glm::max(result.hi, mapped);
    }
    return result;
}

Frustum::Frustum(const glm::mat4& viewProjection) {
    // A point is visible exactly when its clip coordinates satisfy
    // -w <= x <= w, -w <= y <= w and 0 <= z <= w (Vulkan depth). Each plane below
    // is one of those inequalities rewritten with the inside side at >= 0.
    const glm::vec4 rx = clipRow(0, viewProjection);
    const glm::vec4 ry = clipRow(1, viewProjection);
    const glm::vec4 rz = clipRow(2, viewProjection);
    const glm::vec4 rw = clipRow(3, viewProjection);

    planes_[kLeft] = rx + rw;
    planes_[kRight] = -rx + rw;
    planes_[kBottom] = ry + rw;
    planes_[kTop] = -ry + rw;
    // The projection is reversed-Z over Vulkan's [0, 1] depth (near maps to 1, far
    // to 0), so the near plane is z <= w and the far plane is z >= 0 -- the
    // opposite pairing to a conventional GL projection.
    planes_[kNear] = rw - rz;
    planes_[kFar] = rz;

    for (glm::vec4& plane : planes_) {
        normalize(plane);
    }
}

float Frustum::distance(int plane, const glm::vec3& point) const {
    return dot(planes_[plane], point);
}

bool Frustum::contains(const glm::vec3& center, float radius) const {
    for (const glm::vec4& plane : planes_) {
        if (dot(plane, center) < -radius) {
            return false;
        }
    }
    return true;
}

bool Frustum::contains(const Aabb& box) const {
    for (const glm::vec4& plane : planes_) {
        // The corner furthest along the plane normal: if even that one is behind
        // the plane, the whole box is.
        const glm::vec3 positive(plane.x >= 0.0f ? box.hi.x : box.lo.x, plane.y >= 0.0f ? box.hi.y : box.lo.y,
                                 plane.z >= 0.0f ? box.hi.z : box.lo.z);
        if (dot(plane, positive) < 0.0f) {
            return false;
        }
    }
    return true;
}

} // namespace sim::render
