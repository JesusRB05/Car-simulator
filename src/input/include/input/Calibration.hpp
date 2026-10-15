// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

namespace sim::input {

// Per-axis calibration. Raw readings arrive normalised to [-1, 1] by the
// backend, but real wheels are never perfect: the centre drifts, the ends stop
// short or overshoot, and the pedals often rest at one extreme. The calibration
// records the measured extremes and turns any raw reading into a clean value.
struct AxisCalibration {
    double rawMin = -1.0;    // raw reading at the axis' minimum mechanically
    double rawMax = 1.0;     // raw reading at the axis' maximum
    double center = 0.0;     // raw reading at neutral (signed axes only)
    double deadzone = 0.0;   // [0, 1): fraction ignored around centre / at rest
    double saturation = 0.0; // [0, 1): fraction trimmed before reaching full
    bool invert = false;     // flip the direction of travel
    bool useCenter = true;   // false = pedals, map rawMin..rawMax straight to 0..1
    bool active = true;      // false = axis not used (disabled binding)
};

// Signed control in [-1, 1], 0 at centre: steering, or a joystick X axis.
double toSigned(double raw, const AxisCalibration& calibration);

// Unit control in [0, 1], 0 at `rawMin`: pedals and triggers.
double toUnit(double raw, const AxisCalibration& calibration);

// Shapes an already-normalised [0, 1] value by the deadzone (at the 0 end) and
// the saturation (at the 1 end). Used for split / combined pedal halves where
// the raw value is taken straight from a half of a centred axis.
double shapeUnit(double value, double deadzone, double saturation);

} // namespace sim::input
