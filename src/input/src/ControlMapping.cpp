// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "input/ControlMapping.hpp"

namespace sim::input {

const char* toString(ControlRole role) {
    switch (role) {
    case ControlRole::Steer:
        return "Steering";
    case ControlRole::Throttle:
        return "Throttle";
    case ControlRole::Brake:
        return "Brake";
    case ControlRole::Clutch:
        return "Clutch";
    case ControlRole::Handbrake:
        return "Handbrake";
    case ControlRole::ShiftUp:
        return "Shift up";
    case ControlRole::ShiftDown:
        return "Shift down";
    case ControlRole::GearNeutral:
        return "Neutral";
    case ControlRole::LookLeft:
        return "Look left";
    case ControlRole::LookRight:
        return "Look right";
    case ControlRole::Count:
        break;
    }
    return "?";
}

const ControlBinding* DeviceProfile::find(ControlRole role) const {
    const auto it = bindings.find(role);
    return it == bindings.end() ? nullptr : &it->second;
}

ControlBinding& DeviceProfile::ensure(ControlRole role) {
    return bindings[role];
}

namespace {

void bindAxis(DeviceProfile& profile, ControlRole role, int axis, AxisHalf half) {
    ControlBinding binding;
    binding.kind = BindingKind::Axis;
    binding.axis.axis = axis;
    binding.axis.half = half;
    if (role == ControlRole::Throttle || role == ControlRole::Brake || role == ControlRole::Clutch) {
        binding.axis.calibration.useCenter = half == AxisHalf::Full ? false : true;
    }
    profile.bindings[role] = binding;
}

void bindButton(DeviceProfile& profile, ControlRole role, int button) {
    ControlBinding binding;
    binding.kind = BindingKind::Button;
    binding.button = button;
    profile.bindings[role] = binding;
}

} // namespace

DeviceProfile defaultProfile(const DeviceDescriptor& device, DeviceKind kind) {
    DeviceProfile profile;
    profile.key = deviceKey(device);
    profile.modelName = device.name;
    const int axes = device.caps.axes;
    const int buttons = device.caps.buttons;

    switch (kind) {
    case DeviceKind::Pedals:
        if (axes > 0) {
            bindAxis(profile, ControlRole::Throttle, 0, AxisHalf::Full);
        }
        if (axes > 1) {
            bindAxis(profile, ControlRole::Brake, 1, AxisHalf::Full);
        }
        if (axes > 2) {
            bindAxis(profile, ControlRole::Clutch, 2, AxisHalf::Full);
        }
        break;
    case DeviceKind::Shifter:
        if (buttons > 0) {
            bindButton(profile, ControlRole::ShiftDown, 0);
        }
        if (buttons > 1) {
            bindButton(profile, ControlRole::ShiftUp, 1);
        }
        break;
    case DeviceKind::Handbrake:
        if (axes > 0) {
            bindAxis(profile, ControlRole::Handbrake, 0, AxisHalf::Full);
        } else if (buttons > 0) {
            bindButton(profile, ControlRole::Handbrake, 0);
        }
        break;
    case DeviceKind::Gamepad:
        // SDL standard layout: left stick X steers, the second stick's vertical
        // axis holds throttle (down) and brake (up), triggers can be added later.
        if (axes > 0) {
            bindAxis(profile, ControlRole::Steer, 0, AxisHalf::Full);
        }
        if (axes > 1) {
            bindAxis(profile, ControlRole::Throttle, 1, AxisHalf::NegativeHalf);
            bindAxis(profile, ControlRole::Brake, 1, AxisHalf::PositiveHalf);
        }
        break;
    case DeviceKind::WheelBase:
    case DeviceKind::GenericHid:
    case DeviceKind::Unknown:
    default:
        if (axes > 0) {
            bindAxis(profile, ControlRole::Steer, 0, AxisHalf::Full);
        }
        if (axes > 1) {
            bindAxis(profile, ControlRole::Throttle, 1, AxisHalf::Full);
        }
        if (axes > 2) {
            bindAxis(profile, ControlRole::Brake, 2, AxisHalf::Full);
        }
        if (axes > 3) {
            bindAxis(profile, ControlRole::Clutch, 3, AxisHalf::Full);
        }
        if (buttons > 0) {
            bindButton(profile, ControlRole::GearNeutral, 0);
        }
        break;
    }

    return profile;
}

} // namespace sim::input
