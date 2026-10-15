// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "input/Calibration.hpp"

#include <algorithm>
#include <cmath>

namespace sim::input {

namespace {

double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

double saturate(double value, double saturation) {
    const double cap = std::max(1e-6, 1.0 - clamp(saturation, 0.0, 0.99));
    return clamp(value / cap, -1.0, 1.0);
}

} // namespace

double shapeUnit(double value, double deadzone, double saturation) {
    const double dz = clamp(deadzone, 0.0, 0.99);
    double out = clamp(value, 0.0, 1.0);
    if (out <= dz) {
        return 0.0;
    }
    out = (out - dz) / (1.0 - dz);
    const double cap = std::max(1e-6, 1.0 - clamp(saturation, 0.0, 0.99));
    return clamp(out / cap, 0.0, 1.0);
}

double toSigned(double raw, const AxisCalibration& calibration) {
    if (!calibration.active) {
        return 0.0;
    }
    const double lo = std::min(calibration.rawMin, calibration.rawMax);
    const double hi = std::max(calibration.rawMin, calibration.rawMax);
    if (hi - lo < 1e-9) {
        return 0.0;
    }

    double unit = clamp((raw - lo) / (hi - lo), 0.0, 1.0);
    const double center = clamp((calibration.center - lo) / (hi - lo), 0.0, 1.0);

    double signed01 = 0.0;
    if (unit >= center) {
        const double span = std::max(1.0 - center, 1e-6);
        signed01 = (unit - center) / span;
    } else {
        const double span = std::max(center, 1e-6);
        signed01 = -(center - unit) / span;
    }

    const double dz = clamp(calibration.deadzone, 0.0, 0.99);
    const double magnitude = std::abs(signed01);
    if (magnitude <= dz) {
        return 0.0;
    }
    double shaped = std::copysign((magnitude - dz) / (1.0 - dz), signed01);
    shaped = saturate(shaped, calibration.saturation);
    return calibration.invert ? -shaped : shaped;
}

double toUnit(double raw, const AxisCalibration& calibration) {
    if (!calibration.active) {
        return 0.0;
    }
    const double lo = std::min(calibration.rawMin, calibration.rawMax);
    const double hi = std::max(calibration.rawMin, calibration.rawMax);
    if (hi - lo < 1e-9) {
        return 0.0;
    }
    double unit = clamp((raw - lo) / (hi - lo), 0.0, 1.0);
    if (calibration.invert) {
        unit = 1.0 - unit;
    }
    return shapeUnit(unit, calibration.deadzone, calibration.saturation);
}

} // namespace sim::input
