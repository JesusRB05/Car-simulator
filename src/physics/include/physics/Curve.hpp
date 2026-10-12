// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::physics {

// A scalar function y = f(x) defined by a set of control points and evaluated
// with piecewise-linear interpolation. Outside the point range the value is
// clamped to the nearest endpoint, keeping force curves well behaved. An empty
// curve evaluates to y = slope * x (used for ordinary linear springs).
class Curve
{
public:
    struct Point
    {
        Real x = 0.0;
        Real y = 0.0;
    };

    Curve() = default;

    static Curve linear(Real slope);

    void addPoint(Real x, Real y);
    void setPoints(std::vector<Point> points);

    Real evaluate(Real x) const;

    bool empty() const { return points_.empty(); }
    const std::vector<Point>& points() const { return points_; }

private:
    std::vector<Point> points_;
    Real slope_ = 0.0;
};

} // namespace sim::physics
