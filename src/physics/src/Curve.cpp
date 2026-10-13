// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/Curve.hpp"

#include <algorithm>

namespace sim::physics {

Curve Curve::linear(Real slope) {
    Curve curve;
    curve.slope_ = slope;
    return curve;
}

void Curve::addPoint(Real x, Real y) {
    points_.push_back(Point{x, y});
    std::sort(points_.begin(), points_.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
}

void Curve::setPoints(std::vector<Point> points) {
    points_ = std::move(points);
    std::sort(points_.begin(), points_.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
}

Real Curve::evaluate(Real x) const {
    if (points_.empty()) {
        return slope_ * x;
    }
    if (x <= points_.front().x) {
        return points_.front().y;
    }
    if (x >= points_.back().x) {
        return points_.back().y;
    }
    for (std::size_t i = 1; i < points_.size(); ++i) {
        if (x <= points_[i].x) {
            const Point& a = points_[i - 1];
            const Point& b = points_[i];
            const Real span = b.x - a.x;
            if (span <= 0.0) {
                return b.y;
            }
            const Real t = (x - a.x) / span;
            return a.y + (b.y - a.y) * t;
        }
    }
    return points_.back().y;
}

} // namespace sim::physics
