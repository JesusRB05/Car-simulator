// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "input/WheelManager.hpp"

#include <algorithm>
#include <climits>
#include <utility>

#include "core/Log.hpp"

namespace sim::input {

namespace {

bool isPedalRole(ControlRole role) {
    return role == ControlRole::Throttle || role == ControlRole::Brake || role == ControlRole::Clutch;
}

} // namespace

WheelManager::WheelManager(IInputBackend& backend, CompatibilityDatabase& database, ProfileStore& profiles)
    : backend_(backend), database_(database), profiles_(profiles) {}

WheelManager::~WheelManager() {
    stopForceFeedback();
}

void WheelManager::update(double dt) {
    reconcileDevices();
    resolveControls();
    updateFfb(dt);
}

void WheelManager::reconcileDevices() {
    backend_.poll();
    const std::vector<DeviceDescriptor> current = backend_.deviceDescriptors();

    // Drop devices that are gone.
    for (auto it = devices_.begin(); it != devices_.end();) {
        const bool present = std::any_of(current.begin(), current.end(), [&](const DeviceDescriptor& d) {
            return d.instanceId == it->descriptor.instanceId;
        });
        if (present) {
            ++it;
            continue;
        }
        // A device disappeared: the backend has already torn down its effects,
        // but ask once more in case the output object is still alive.
        if (IFfbOutput* output = backend_.ffbOutput(it->descriptor.instanceId); output != nullptr) {
            output->stopAll();
        }
        core::Log::get("input")->info("removed '{}' ({})", it->descriptor.name, it->identification.modelName);
        ++removedCount_;
        it = devices_.erase(it);
    }

    // Add new devices.
    for (const DeviceDescriptor& descriptor : current) {
        const bool known = std::any_of(devices_.begin(), devices_.end(), [&](const DeviceEntry& e) {
            return e.descriptor.instanceId == descriptor.instanceId;
        });
        if (known) {
            continue;
        }

        DeviceEntry entry;
        entry.descriptor = descriptor;
        entry.identification = database_.identify(descriptor);
        if (entry.identification.manufacturer.empty() && !descriptor.manufacturer.empty()) {
            entry.identification.manufacturer = descriptor.manufacturer;
        }
        const DeviceProfile fallback = defaultProfile(descriptor, entry.identification.kind);
        entry.profile = profiles_.loadProfile(deviceKey(descriptor), fallback);
        devices_.push_back(std::move(entry));

        const DeviceEntry& added = devices_.back();
        core::Log::get("input")->info("connected '{}' -> {} {} ({} axes, {} buttons, {} hats{})", added.descriptor.name,
                                      added.identification.manufacturer, added.identification.modelName,
                                      added.descriptor.caps.axes, added.descriptor.caps.buttons,
                                      added.descriptor.caps.hats, added.identification.known() ? "" : ", unknown");
        if (!added.identification.known()) {
            core::Log::get("input")->warn("  '{}' is not in the compatibility database; using a generic layout",
                                          added.descriptor.name);
        }
    }

    diagnostics_.removedDevices = removedCount_;
    diagnostics_.connectedDevices = static_cast<int>(devices_.size());
}

int WheelManager::primaryDeviceIndex() const {
    if (selected_ >= 0 && selected_ < static_cast<int>(devices_.size())) {
        return selected_;
    }
    for (std::size_t i = 0; i < devices_.size(); ++i) {
        const DeviceKind kind = devices_[i].identification.kind;
        if (kind == DeviceKind::WheelBase || kind == DeviceKind::Gamepad) {
            return static_cast<int>(i);
        }
    }
    return devices_.empty() ? -1 : 0;
}

int WheelManager::rolePriority(std::size_t deviceIndex, ControlRole role) const {
    if (static_cast<int>(deviceIndex) == primaryDeviceIndex()) {
        return 0;
    }
    const DeviceKind kind = devices_[deviceIndex].identification.kind;
    if (isPedalRole(role) && kind == DeviceKind::Pedals) {
        return 1;
    }
    if ((role == ControlRole::ShiftUp || role == ControlRole::ShiftDown) && kind == DeviceKind::Shifter) {
        return 1;
    }
    if (role == ControlRole::Handbrake && kind == DeviceKind::Handbrake) {
        return 1;
    }
    return 2;
}

double WheelManager::evaluate(const DeviceEntry& device, ControlRole role, const ControlBinding& binding, bool& bound) {
    bound = false;
    switch (binding.kind) {
    case BindingKind::Axis: {
        const int axis = binding.axis.axis;
        if (axis < 0 || axis >= static_cast<int>(device.state.axes.size())) {
            return 0.0;
        }
        bound = true;
        const double raw = device.state.axes[static_cast<std::size_t>(axis)];
        const AxisCalibration& calibration = binding.axis.calibration;
        if (binding.axis.half == AxisHalf::Full) {
            if (role == ControlRole::Steer) {
                return toSigned(raw, calibration);
            }
            return toUnit(raw, calibration);
        }
        double unit =
            binding.axis.half == AxisHalf::NegativeHalf ? std::clamp(-raw, 0.0, 1.0) : std::clamp(raw, 0.0, 1.0);
        if (calibration.invert) {
            unit = 1.0 - unit;
        }
        return shapeUnit(unit, calibration.deadzone, calibration.saturation);
    }
    case BindingKind::Button: {
        const int button = binding.button;
        if (button < 0 || button >= static_cast<int>(device.state.buttons.size())) {
            return 0.0;
        }
        bound = true;
        return device.state.buttons[static_cast<std::size_t>(button)] ? 1.0 : 0.0;
    }
    case BindingKind::Hat: {
        const int hat = binding.hat;
        if (hat < 0 || hat >= static_cast<int>(device.state.hats.size())) {
            return 0.0;
        }
        bound = true;
        return (device.state.hats[static_cast<std::size_t>(hat)] & binding.hatDirection) != 0 ? 1.0 : 0.0;
    }
    case BindingKind::None:
        break;
    }
    return 0.0;
}

void WheelManager::resolveControls() {
    controls_ = Controls{};
    diagnostics_.unknownDevices = 0;

    for (DeviceEntry& device : devices_) {
        backend_.readState(device.descriptor.instanceId, device.state);
        if (!device.identification.known()) {
            ++diagnostics_.unknownDevices;
        }
        const DeviceKind kind = device.identification.kind;
        if (kind == DeviceKind::WheelBase || kind == DeviceKind::Gamepad) {
            controls_.hasDrivingDevice = true;
        }
    }

    const auto pick = [&](ControlRole role, double& out) {
        int bestPriority = INT_MAX;
        double bestValue = 0.0;
        bool found = false;
        for (std::size_t i = 0; i < devices_.size(); ++i) {
            const ControlBinding* binding = devices_[i].profile.find(role);
            if (binding == nullptr) {
                continue;
            }
            bool bound = false;
            const double value = evaluate(devices_[i], role, *binding, bound);
            if (!bound) {
                continue;
            }
            const int priority = rolePriority(i, role);
            if (priority < bestPriority) {
                bestPriority = priority;
                bestValue = value;
                found = true;
            }
        }
        if (found) {
            out = bestValue;
        }
        return found;
    };

    pick(ControlRole::Steer, controls_.steer);
    pick(ControlRole::Throttle, controls_.throttle);
    pick(ControlRole::Brake, controls_.brake);
    pick(ControlRole::Clutch, controls_.clutch);
    pick(ControlRole::Handbrake, controls_.handbrake);
    controls_.steer = std::clamp(controls_.steer, -1.0, 1.0);

    const auto pressed = [&](ControlRole role) {
        for (std::size_t i = 0; i < devices_.size(); ++i) {
            const ControlBinding* binding = devices_[i].profile.find(role);
            if (binding == nullptr) {
                continue;
            }
            bool bound = false;
            if (evaluate(devices_[i], role, *binding, bound) > 0.5) {
                return true;
            }
        }
        return false;
    };

    controls_.shiftUp = pressed(ControlRole::ShiftUp);
    controls_.shiftDown = pressed(ControlRole::ShiftDown);
    controls_.gearNeutral = pressed(ControlRole::GearNeutral);
    controls_.lookLeft = pressed(ControlRole::LookLeft);
    controls_.lookRight = pressed(ControlRole::LookRight);
}

void WheelManager::updateFfb(double dt) {
    diagnostics_.ffbActive = false;
    diagnostics_.ffbAvailable = false;

    IFfbOutput* output = nullptr;
    const int primary = primaryDeviceIndex();
    if (primary >= 0) {
        output = backend_.ffbOutput(devices_[static_cast<std::size_t>(primary)].descriptor.instanceId);
    }
    if (output == nullptr) {
        for (DeviceEntry& device : devices_) {
            output = backend_.ffbOutput(device.descriptor.instanceId);
            if (output != nullptr) {
                break;
            }
        }
    }

    if (output == nullptr) {
        ffb_.settings().caps = FfbCapabilities{};
        ffb_.update(ffbInput_, /*devicePresent=*/false, backend_.windowFocused(), dt);
        return;
    }

    const FfbCapabilities caps = output->capabilities();
    diagnostics_.ffbAvailable = caps.any();
    ffb_.settings().caps = caps;

    const FfbCommand command = ffb_.update(ffbInput_, /*devicePresent=*/true, backend_.windowFocused(), dt);
    if (!command.active) {
        output->stopAll();
        return;
    }

    diagnostics_.ffbActive = true;
    if (caps.constantForce) {
        output->setConstantForce(command.constantForce);
    }
    if (caps.spring) {
        output->setCondition(FfbCondition::Spring, command.spring);
    }
    if (caps.damper) {
        output->setCondition(FfbCondition::Damper, command.damper);
    }
    if (caps.friction) {
        output->setCondition(FfbCondition::Friction, command.friction);
    }
    if (caps.rumble) {
        output->setRumble(command.rumbleLow, command.rumbleHigh, 0);
    }
}

void WheelManager::stopForceFeedback() {
    for (DeviceEntry& device : devices_) {
        if (IFfbOutput* output = backend_.ffbOutput(device.descriptor.instanceId); output != nullptr) {
            output->stopAll();
        }
    }
    ffb_.requestStop();
}

void WheelManager::bindAxis(std::size_t deviceIndex, ControlRole role, int axis, AxisHalf half) {
    if (deviceIndex >= devices_.size()) {
        return;
    }
    ControlBinding binding;
    binding.kind = BindingKind::Axis;
    binding.axis.axis = axis;
    binding.axis.half = half;
    binding.axis.calibration.useCenter = half == AxisHalf::Full && role == ControlRole::Steer;
    devices_[deviceIndex].profile.bindings[role] = binding;
}

void WheelManager::bindButton(std::size_t deviceIndex, ControlRole role, int button) {
    if (deviceIndex >= devices_.size()) {
        return;
    }
    ControlBinding binding;
    binding.kind = BindingKind::Button;
    binding.button = button;
    devices_[deviceIndex].profile.bindings[role] = binding;
}

void WheelManager::bindHat(std::size_t deviceIndex, ControlRole role, int hat, int direction) {
    if (deviceIndex >= devices_.size()) {
        return;
    }
    ControlBinding binding;
    binding.kind = BindingKind::Hat;
    binding.hat = hat;
    binding.hatDirection = direction;
    devices_[deviceIndex].profile.bindings[role] = binding;
}

void WheelManager::clearBinding(std::size_t deviceIndex, ControlRole role) {
    if (deviceIndex >= devices_.size()) {
        return;
    }
    devices_[deviceIndex].profile.bindings.erase(role);
}

void WheelManager::setCalibration(std::size_t deviceIndex, ControlRole role, const AxisCalibration& calibration) {
    if (deviceIndex >= devices_.size()) {
        return;
    }
    ControlBinding& binding = devices_[deviceIndex].profile.ensure(role);
    if (binding.kind == BindingKind::Axis) {
        binding.axis.calibration = calibration;
    }
}

void WheelManager::setSteeringRange(std::size_t deviceIndex, double degrees) {
    if (deviceIndex >= devices_.size()) {
        return;
    }
    devices_[deviceIndex].profile.steeringRangeDegrees = degrees;
}

void WheelManager::storeProfiles() {
    for (const DeviceEntry& device : devices_) {
        profiles_.storeProfile(device.profile);
    }
    profiles_.storeFfbSettings(ffb_.settings());
}

} // namespace sim::input
