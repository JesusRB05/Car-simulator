// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <map>
#include <string>

#include "input/Calibration.hpp"
#include "input/Types.hpp"

namespace sim::input {

// The abstract controls a driving game cares about. Everything a wheel, a
// pedal set, a shifter or a gamepad can offer maps onto one of these.
enum class ControlRole {
    Steer,
    Throttle,
    Brake,
    Clutch,
    Handbrake,
    ShiftUp,
    ShiftDown,
    GearNeutral,
    LookLeft,
    LookRight,
    Count,
};

const char* toString(ControlRole role);

enum class BindingKind {
    None, // unbound
    Axis,
    Button,
    Hat,
};

// A pedal axis may occupy the whole axis (a dedicated pedal) or one half of a
// centred axis (the classic "combined pedals" mode, where one axis holds both
// throttle and brake).
enum class AxisHalf {
    Full,         // signed axis (steering) or a dedicated single-direction pedal
    NegativeHalf, // the negative half of a centred axis
    PositiveHalf, // the positive half of a centred axis
};

struct AxisBinding {
    int axis = -1;
    AxisHalf half = AxisHalf::Full;
    AxisCalibration calibration{};
};

struct ControlBinding {
    BindingKind kind = BindingKind::None;
    AxisBinding axis{};
    int button = -1;
    int hat = -1;
    int hatDirection = 0; // SDL_HAT_* bitmask (1=UP, 2=RIGHT, 4=DOWN, 8=LEFT)
};

// Everything that makes one physical device behave: which input feeds which
// role and how it is calibrated. One profile per device, saved under the
// device's stable key.
struct DeviceProfile {
    int version = 1;
    std::string key;       // stable device key ("046d:c29b" or a name slug)
    std::string modelName; // for display only

    // Physical rotation of the steering axis from lock to lock, in degrees
    // (900 is the common default). Used for the on-screen wheel and the force
    // feedback feel; it never changes how much the car actually steers.
    double steeringRangeDegrees = 900.0;

    std::map<ControlRole, ControlBinding> bindings;

    const ControlBinding* find(ControlRole role) const;
    ControlBinding& ensure(ControlRole role);
};

// Builds a sensible starting profile for a freshly seen device. The layout is
// driven by the device kind and the number of axes / buttons it reports; the
// user can remap everything afterwards.
DeviceProfile defaultProfile(const DeviceDescriptor& device, DeviceKind kind);

// Name used in menus and profiles for a role.
inline const char* roleName(ControlRole role) {
    return toString(role);
}

} // namespace sim::input
