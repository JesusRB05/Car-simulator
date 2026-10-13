// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Engine.hpp"

#include <cstddef>

namespace sim::vehicle {

namespace {

// Locates x within an increasing grid and returns the bracketing indices and the
// fraction between them. Values off the ends are clamped (constant extrapolation).
void locate(const std::vector<Real>& grid, Real x, std::size_t& i0, std::size_t& i1, Real& t) {
    const std::size_t n = grid.size();
    if (n == 1) {
        i0 = 0;
        i1 = 0;
        t = 0.0;
        return;
    }
    if (x <= grid.front()) {
        i0 = 0;
        i1 = 1;
        t = 0.0;
        return;
    }
    if (x >= grid.back()) {
        i0 = n - 2;
        i1 = n - 1;
        t = 1.0;
        return;
    }
    i0 = 0;
    while (i0 + 1 < n && x > grid[i0 + 1]) {
        ++i0;
    }
    i1 = i0 + 1;
    t = (x - grid[i0]) / (grid[i1] - grid[i0]);
}

} // namespace

bool EngineTorqueMap::empty() const {
    return speed.empty() || throttle.empty() || torque.size() != speed.size() * throttle.size();
}

Real EngineTorqueMap::torqueAt(Real angularVelocity, Real throttlePosition) const {
    if (empty()) {
        return 0.0;
    }
    std::size_t i0 = 0;
    std::size_t i1 = 0;
    std::size_t j0 = 0;
    std::size_t j1 = 0;
    Real ts = 0.0;
    Real tt = 0.0;
    locate(speed, angularVelocity, i0, i1, ts);
    locate(throttle, throttlePosition, j0, j1, tt);

    const std::size_t nt = throttle.size();
    const Real v00 = torque[i0 * nt + j0];
    const Real v01 = torque[i0 * nt + j1];
    const Real v10 = torque[i1 * nt + j0];
    const Real v11 = torque[i1 * nt + j1];

    const Real bottom = v00 + tt * (v01 - v00);
    const Real top = v10 + tt * (v11 - v10);
    return bottom + ts * (top - bottom);
}

Real EngineFriction::torqueAt(Real angularVelocity) const {
    return constant + linear * angularVelocity + quadratic * angularVelocity * angularVelocity;
}

} // namespace sim::vehicle
