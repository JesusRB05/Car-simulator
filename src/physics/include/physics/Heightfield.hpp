// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/Math.hpp"
#include "physics/PhysicsTypes.hpp"

namespace sim::physics {

// A regular height grid over the XZ plane (Phase 1, P1.3). Heights are sampled on
// a uniform lattice and the surface between samples is triangulated (each cell
// splits along its (0,0)-(1,1) diagonal), so queries and contacts share one
// well-defined surface. Samples are row-major: index (ix, iz) is iz*width + ix.
struct Heightfield {
    int width = 0;      // samples along +X
    int depth = 0;      // samples along +Z
    Real spacing = 1.0; // metres between neighbouring samples
    Vec3 origin{0.0};   // world position of sample (0, 0); only x/z are used
    std::vector<Real> heights;

    bool valid() const { return width >= 2 && depth >= 2 && heights.size() == static_cast<std::size_t>(width) * depth; }

    Real heightAt(int ix, int iz) const { return heights[static_cast<std::size_t>(iz) * width + ix]; }

    // True when (x, z) lies inside the grid footprint (inclusive of the edges).
    bool contains(Real x, Real z) const;

    // Height of the triangulated surface at a world (x, z). Outside the footprint
    // the nearest edge sample is used (the surface is extended flat), so the
    // query is always defined.
    Real sampleHeight(Real x, Real z) const;

    // Unit upward normal of the triangulated surface at a world (x, z).
    Vec3 sampleNormal(Real x, Real z) const;

    // World-space bounds and footprint (min/max height scanned from the grid).
    Vec3 minCorner() const;
    Vec3 maxCorner() const;
};

} // namespace sim::physics
