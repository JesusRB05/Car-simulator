// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Heightfield.hpp"

#include <algorithm>
#include <cmath>

namespace sim::physics {

bool Heightfield::contains(Real x, Real z) const {
    if (!valid()) {
        return false;
    }
    const Real maxX = origin.x + static_cast<Real>(width - 1) * spacing;
    const Real maxZ = origin.z + static_cast<Real>(depth - 1) * spacing;
    return x >= origin.x && x <= maxX && z >= origin.z && z <= maxZ;
}

Real Heightfield::sampleHeight(Real x, Real z) const {
    if (!valid()) {
        return origin.y;
    }
    // Map to fractional lattice coordinates and clamp to the footprint so the
    // surface extends flat beyond the sampled region.
    Real fx = (x - origin.x) / spacing;
    Real fz = (z - origin.z) / spacing;
    fx = std::clamp(fx, 0.0, static_cast<Real>(width - 1));
    fz = std::clamp(fz, 0.0, static_cast<Real>(depth - 1));
    const int ix = std::min(static_cast<int>(std::floor(fx)), width - 2);
    const int iz = std::min(static_cast<int>(std::floor(fz)), depth - 2);
    const Real u = std::clamp(fx - static_cast<Real>(ix), 0.0, 1.0);
    const Real v = std::clamp(fz - static_cast<Real>(iz), 0.0, 1.0);
    const Real h00 = heightAt(ix, iz);
    const Real h10 = heightAt(ix + 1, iz);
    const Real h01 = heightAt(ix, iz + 1);
    const Real h11 = heightAt(ix + 1, iz + 1);
    if (u + v <= 1.0) {
        return h00 + u * (h10 - h00) + v * (h11 - h10);
    }
    return h00 + u * (h11 - h01) + v * (h01 - h00);
}

Vec3 Heightfield::sampleNormal(Real x, Real z) const {
    if (!valid()) {
        return Vec3(0.0, 1.0, 0.0);
    }
    Real fx = (x - origin.x) / spacing;
    Real fz = (z - origin.z) / spacing;
    fx = std::clamp(fx, 0.0, static_cast<Real>(width - 1));
    fz = std::clamp(fz, 0.0, static_cast<Real>(depth - 1));
    const int ix = std::min(static_cast<int>(std::floor(fx)), width - 2);
    const int iz = std::min(static_cast<int>(std::floor(fz)), depth - 2);
    const Real u = std::clamp(fx - static_cast<Real>(ix), 0.0, 1.0);
    const Real v = std::clamp(fz - static_cast<Real>(iz), 0.0, 1.0);
    const Real h00 = heightAt(ix, iz);
    const Real h10 = heightAt(ix + 1, iz);
    const Real h01 = heightAt(ix, iz + 1);
    const Real h11 = heightAt(ix + 1, iz + 1);
    // Height gradient of the active triangle; the normal is (-dh/dx, 1, -dh/dz).
    Real dhdx = 0.0;
    Real dhdz = 0.0;
    if (u + v <= 1.0) {
        dhdx = h10 - h00;
        dhdz = h11 - h10;
    } else {
        dhdx = h11 - h01;
        dhdz = h01 - h00;
    }
    return safeNormalize(Vec3(-dhdx / spacing, 1.0, -dhdz / spacing));
}

Vec3 Heightfield::minCorner() const {
    if (!valid()) {
        return origin;
    }
    Real lowest = heightAt(0, 0);
    for (Real h : heights) {
        lowest = std::min(lowest, h);
    }
    return Vec3(origin.x, lowest, origin.z);
}

Vec3 Heightfield::maxCorner() const {
    if (!valid()) {
        return origin;
    }
    Real highest = heightAt(0, 0);
    for (Real h : heights) {
        highest = std::max(highest, h);
    }
    return Vec3(origin.x + static_cast<Real>(width - 1) * spacing, highest,
                origin.z + static_cast<Real>(depth - 1) * spacing);
}

} // namespace sim::physics
