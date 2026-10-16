// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <glm/glm.hpp>

namespace sim::render {

// An axis-aligned box, used as the culling volume of a drawable.
struct Aabb {
    glm::vec3 lo{0.0f};
    glm::vec3 hi{0.0f};
};

// The world-space bounds of an axis-aligned box pushed through an affine
// transform: all eight corners are mapped and re-fitted, which is exact for any
// translation, rotation, uniform or non-uniform scale (all the app produces).
Aabb transformBounds(const Aabb& local, const glm::mat4& transform);

// The visible volume of a view-projection matrix, so the renderer can skip
// drawing geometry the camera cannot see. Every plane is stored normalised with
// its inside half-space at distance() >= 0.
//
// Culling is conservative: `contains()` returns true for anything that touches
// the volume, so a miss only ever costs an overdraw that was not needed, never a
// wrongly hidden object (see the conservativeness test in test_render_math).
class Frustum {
public:
    explicit Frustum(const glm::mat4& viewProjection);

    // Signed distance from `point` to plane `plane` (0..5), >= 0 meaning inside.
    float distance(int plane, const glm::vec3& point) const;

    // True when any part of the sphere is inside the frustum.
    bool contains(const glm::vec3& center, float radius) const;

    // True when any part of the box is inside the frustum.
    bool contains(const Aabb& box) const;

    static constexpr int kPlaneCount = 6;
    static constexpr int kLeft = 0;
    static constexpr int kRight = 1;
    static constexpr int kBottom = 2;
    static constexpr int kTop = 3;
    static constexpr int kNear = 4;
    static constexpr int kFar = 5;

private:
    glm::vec4 planes_[kPlaneCount]{};
};

} // namespace sim::render
