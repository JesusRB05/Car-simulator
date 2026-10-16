// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <vector>

#include "physics/Math.hpp"
#include "physics/PhysicsTypes.hpp"

namespace sim::physics {

// A static triangle soup (Phase 1, P1.3). Vertices are stored in the body's local
// frame and transformed by its pose, so an arbitrary mesh can be dropped into the
// world as a static body. Outward-facing triangle winding matters for the box
// contact's inside test.
struct TriangleMesh {
    std::vector<Vec3> vertices;         // local frame
    std::vector<std::uint32_t> indices; // three indices per triangle

    std::size_t triangleCount() const { return indices.size() / 3; }
    bool valid() const { return indices.size() >= 3 && indices.size() % 3 == 0; }

    // The three local vertices of triangle `t`.
    void triangle(std::size_t t, Vec3& a, Vec3& b, Vec3& c) const {
        a = vertices[indices[3 * t + 0]];
        b = vertices[indices[3 * t + 1]];
        c = vertices[indices[3 * t + 2]];
    }
};

} // namespace sim::physics
