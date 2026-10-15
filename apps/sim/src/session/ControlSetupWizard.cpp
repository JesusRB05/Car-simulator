// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/ControlSetupWizard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <imgui.h>

#include "input/Calibration.hpp"
#include "input/ControlMapping.hpp"
#include "input/Types.hpp"
#include "input/WheelManager.hpp"

namespace sim::app {
namespace {

using input::AxisHalf;
using input::BindingKind;
using input::ControlBinding;
using input::ControlRole;
using input::DeviceEntry;

constexpr int kStepCount = 6;

const ControlRole kButtonRoles[] = {ControlRole::ShiftUp,   ControlRole::ShiftDown, ControlRole::GearNeutral,
                                    ControlRole::Handbrake, ControlRole::LookLeft,  ControlRole::LookRight};
constexpr int kButtonRoleCount = static_cast<int>(sizeof(kButtonRoles) / sizeof(kButtonRoles[0]));

// The three analog pedals, always in this order.
const ControlRole kPedalRoles[] = {ControlRole::Throttle, ControlRole::Brake, ControlRole::Clutch};
constexpr int kPedalRoleCount = static_cast<int>(sizeof(kPedalRoles) / sizeof(kPedalRoles[0]));

bool sliderDouble(const char* label, double& value, double low, double high, const char* format = "%.2f") {
    return ImGui::SliderScalar(label, ImGuiDataType_Double, &value, &low, &high, format);
}

std::string bindingLabel(const ControlBinding* binding) {
    if (binding == nullptr || binding->kind == BindingKind::None) {
        return "unbound";
    }
    char buffer[48];
    switch (binding->kind) {
    case BindingKind::Axis:
        std::snprintf(buffer, sizeof(buffer), "Axis %d", binding->axis.axis);
        break;
    case BindingKind::Button:
        std::snprintf(buffer, sizeof(buffer), "Button %d", binding->button);
        break;
    case BindingKind::Hat:
        std::snprintf(buffer, sizeof(buffer), "Hat %d", binding->hat);
        break;
    case BindingKind::None:
        std::snprintf(buffer, sizeof(buffer), "unbound");
        break;
    }
    return buffer;
}

const char* ffbLine(const input::FfbCapabilities& caps) {
    if (!caps.any()) {
        return "no force feedback";
    }
    return "force feedback";
}

} // namespace

void ControlSetupWizard::open() {
    step_ = 0;
    lastInitStep_ = -1;
    listeningRole_ = -1;
    conflictRole_ = -1;
}

std::size_t ControlSetupWizard::clampDevice(const input::WheelManager& wheel) const {
    if (wheel.devices().empty()) {
        return 0;
    }
    return std::min(deviceIndex_, wheel.devices().size() - 1);
}

void ControlSetupWizard::beginListening(const input::WheelManager& wheel, int role) {
    listeningRole_ = role;
    conflictRole_ = -1;
    conflictButton_ = -1;
    conflictOther_ = -1;
    if (wheel.devices().empty()) {
        return;
    }
    const DeviceEntry& device = wheel.devices()[clampDevice(wheel)];
    buttonBaseline_ = device.state.buttons;
    hatBaseline_ = device.state.hats;
}

bool ControlSetupWizard::buttonConflict(const input::WheelManager& wheel, int role, int button, int& other) const {
    if (wheel.devices().empty()) {
        return false;
    }
    const DeviceEntry& device = wheel.devices()[clampDevice(wheel)];
    for (const auto& [boundRole, binding] : device.profile.bindings) {
        if (static_cast<int>(boundRole) == role) {
            continue;
        }
        if (binding.kind == BindingKind::Button && binding.button == button) {
            other = static_cast<int>(boundRole);
            return true;
        }
    }
    return false;
}

int ControlSetupWizard::pollListening(input::WheelManager& wheel) {
    if (listeningRole_ < 0 || wheel.devices().empty()) {
        return -1;
    }
    const DeviceEntry& device = wheel.devices()[clampDevice(wheel)];
    // A fresh button press wins over a hat direction.
    for (std::size_t i = 0; i < device.state.buttons.size(); ++i) {
        const bool pressed = i < buttonBaseline_.size() && buttonBaseline_[i];
        if (device.state.buttons[i] && !pressed) {
            int other = -1;
            if (buttonConflict(wheel, listeningRole_, static_cast<int>(i), other)) {
                conflictRole_ = listeningRole_;
                conflictButton_ = static_cast<int>(i);
                conflictOther_ = other;
                listeningRole_ = -1;
                return -1;
            }
            wheel.bindButton(clampDevice(wheel), static_cast<ControlRole>(listeningRole_), static_cast<int>(i));
            const int bound = listeningRole_;
            listeningRole_ = -1;
            return bound;
        }
    }
    for (std::size_t hat = 0; hat < device.state.hats.size(); ++hat) {
        const int before = hat < hatBaseline_.size() ? hatBaseline_[hat] : 0;
        const int now = device.state.hats[hat];
        const int fresh = now & ~before;
        if (fresh != 0) {
            wheel.bindHat(clampDevice(wheel), static_cast<ControlRole>(listeningRole_), static_cast<int>(hat), fresh);
            const int bound = listeningRole_;
            listeningRole_ = -1;
            return bound;
        }
    }
    return -1;
}

void ControlSetupWizard::initStep(input::WheelManager& wheel) {
    if (lastInitStep_ == step_) {
        return;
    }
    lastInitStep_ = step_;
    listeningRole_ = -1;
    conflictRole_ = -1;

    if (wheel.devices().empty()) {
        return;
    }
    const std::size_t index = clampDevice(wheel);
    deviceIndex_ = index;
    const DeviceEntry& device = wheel.devices()[index];

    if (step_ == 2) { // Steering: seed the capture from the stored binding.
        const ControlBinding* binding = device.profile.find(ControlRole::Steer);
        if (binding != nullptr && binding->kind == BindingKind::Axis) {
            steerAxis_ = binding->axis.axis;
            steerMin_ = binding->axis.calibration.rawMin;
            steerMax_ = binding->axis.calibration.rawMax;
            steerCenter_ = binding->axis.calibration.center;
            steerDeadzone_ = binding->axis.calibration.deadzone;
            steerInvert_ = binding->axis.calibration.invert;
            haveMin_ = haveMax_ = haveCenter_ = true;
        } else {
            haveMin_ = haveMax_ = haveCenter_ = false;
        }
        steeringRange_ = device.profile.steeringRangeDegrees;
    } else if (step_ == 3) { // Pedals: capture each axis' rest position.
        const std::size_t axes = device.state.axes.size();
        pedalRest_.assign(axes, 0.0);
        pedalMin_.assign(axes, 0.0);
        pedalMax_.assign(axes, 0.0);
        for (std::size_t a = 0; a < axes; ++a) {
            pedalRest_[a] = device.state.axes[a];
            pedalMin_[a] = device.state.axes[a];
            pedalMax_[a] = device.state.axes[a];
        }
    }
}

void ControlSetupWizard::drawDevices(input::WheelManager& wheel) {
    const input::InputDiagnostics& diag = wheel.diagnostics();
    ImGui::Text("Connected devices: %d", diag.connectedDevices);
    if (diag.removedDevices > 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("(%d removed since start-up)", diag.removedDevices);
    }
    ImGui::TextDisabled("Plug a wheel, pedal set or gamepad at any time: it appears here.");
    ImGui::Separator();

    if (wheel.devices().empty()) {
        ImGui::TextWrapped("No driving device found. You can still drive with the keyboard (W/A/S/D, Space for the "
                           "handbrake).");
        return;
    }

    if (ImGui::BeginTable("devices", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Device");
        ImGui::TableSetupColumn("Kind");
        ImGui::TableSetupColumn("USB");
        ImGui::TableSetupColumn("In use");
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < wheel.devices().size(); ++i) {
            const DeviceEntry& device = wheel.devices()[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::PushID(static_cast<int>(i));
            const bool primary = wheel.primaryDeviceIndex() == static_cast<int>(i);
            if (ImGui::RadioButton(device.descriptor.name.c_str(), primary)) {
                wheel.selectDevice(static_cast<int>(i));
                deviceIndex_ = i;
            }
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(input::toString(device.identification.kind));
            ImGui::TableSetColumnIndex(2);
            if (device.descriptor.vendorId != 0 || device.descriptor.productId != 0) {
                ImGui::Text("%04x:%04x", device.descriptor.vendorId, device.descriptor.productId);
            } else {
                ImGui::TextDisabled("-");
            }
            ImGui::TableSetColumnIndex(3);
            ImGui::TextUnformatted(primary ? "yes" : "");
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::BeginDisabled(wheel.selectedDevice() < 0);
    if (ImGui::Button("Choose automatically")) {
        wheel.selectDevice(-1);
    }
    ImGui::EndDisabled();
}

void ControlSetupWizard::drawCompatibility(input::WheelManager& wheel) {
    if (wheel.devices().empty()) {
        ImGui::TextWrapped("No device to inspect.");
        return;
    }
    const DeviceEntry& device = wheel.devices()[clampDevice(wheel)];
    const input::Identification& id = device.identification;

    ImGui::Text("Device: %s", device.descriptor.name.c_str());
    ImGui::Text("Identified as: %s %s", id.manufacturer.c_str(), id.modelName.c_str());
    ImGui::Text("Kind: %s", input::toString(id.kind));
    ImGui::Text("Compatibility: %s", input::toString(id.status));
    ImGui::Text("Axes: %d   Buttons: %d   Hats: %d", device.descriptor.caps.axes, device.descriptor.caps.buttons,
                device.descriptor.caps.hats);
    ImGui::Separator();
    ImGui::Text("Force feedback: %s", ffbLine(device.descriptor.caps.ffb));
    if (device.descriptor.caps.ffb.any()) {
        const input::FfbCapabilities& caps = device.descriptor.caps.ffb;
        ImGui::BulletText("Rumble: %s", caps.rumble ? "yes" : "no");
        ImGui::BulletText("Trigger rumble: %s", caps.triggerRumble ? "yes" : "no");
        ImGui::BulletText("Constant force: %s", caps.constantForce ? "yes" : "no");
        ImGui::BulletText("Spring / damper / friction: %s", caps.anyForce() ? "yes" : "no");
        ImGui::BulletText("LED / rev lights: %s", caps.led ? "yes" : "no");
    }
    if (!id.known()) {
        ImGui::Spacing();
        ImGui::TextWrapped("This device is not in the compatibility database. It still works, but starts with a "
                           "generic layout you may want to remap in the next steps.");
    }
    if (device.descriptor.caps.requiresVendorDriver) {
        ImGui::TextWrapped("Note: this device may need its vendor software installed to expose all of its features.");
    }
}

void ControlSetupWizard::drawSteering(input::WheelManager& wheel) {
    if (wheel.devices().empty()) {
        ImGui::TextWrapped("No device to calibrate.");
        return;
    }
    const std::size_t index = clampDevice(wheel);
    const DeviceEntry& device = wheel.devices()[index];
    const std::size_t axes = device.state.axes.size();

    ImGui::TextWrapped("Turn the wheel fully left, then fully right, and back to the centre. Capture each position "
                       "while holding it.");
    ImGui::Separator();

    // Axis picker.
    std::string current = "Axis " + std::to_string(steerAxis_);
    if (ImGui::BeginCombo("Steering axis", current.c_str())) {
        for (std::size_t a = 0; a < axes; ++a) {
            const std::string label = "Axis " + std::to_string(a);
            if (ImGui::Selectable(label.c_str(), static_cast<int>(a) == steerAxis_)) {
                steerAxis_ = static_cast<int>(a);
            }
        }
        ImGui::EndCombo();
    }

    const double raw = steerAxis_ >= 0 && steerAxis_ < static_cast<int>(axes) ? device.state.axes[steerAxis_] : 0.0;
    ImGui::Text("Raw value: %+.3f", raw);
    const float bar = static_cast<float>((raw + 1.0) * 0.5);
    ImGui::ProgressBar(bar, ImVec2(-1.0f, 0.0f), "");

    if (ImGui::Button("Capture left")) {
        steerMin_ = raw;
        haveMin_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Capture centre")) {
        steerCenter_ = raw;
        haveCenter_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Capture right")) {
        steerMax_ = raw;
        haveMax_ = true;
    }
    ImGui::Text("Left %s   Centre %s   Right %s", haveMin_ ? "ok" : "-", haveCenter_ ? "ok" : "-",
                haveMax_ ? "ok" : "-");

    ImGui::Spacing();
    sliderDouble("Deadzone", steerDeadzone_, 0.0, 0.3, "%.3f");
    ImGui::Checkbox("Invert steering", &steerInvert_);
    ImGui::SetNextItemWidth(180.0f);
    sliderDouble("Rotation range (deg)", steeringRange_, 180.0, 1080.0, "%.0f");

    // Apply live.
    wheel.bindAxis(index, ControlRole::Steer, steerAxis_, AxisHalf::Full);
    input::AxisCalibration cal;
    cal.rawMin = std::min(steerMin_, steerMax_);
    cal.rawMax = std::max(steerMin_, steerMax_);
    cal.center = steerCenter_;
    cal.deadzone = steerDeadzone_;
    cal.invert = steerInvert_;
    cal.useCenter = true;
    wheel.setCalibration(index, ControlRole::Steer, cal);
    wheel.setSteeringRange(index, steeringRange_);

    const double resolved = wheel.controls().steer;
    ImGui::Separator();
    ImGui::Text("Car steering: %+.2f", resolved);
    ImGui::ProgressBar(static_cast<float>((resolved + 1.0) * 0.5), ImVec2(-1.0f, 0.0f), "");
}

void ControlSetupWizard::drawPedals(input::WheelManager& wheel) {
    if (wheel.devices().empty()) {
        ImGui::TextWrapped("No device to calibrate.");
        return;
    }
    const std::size_t index = clampDevice(wheel);
    const DeviceEntry& device = wheel.devices()[index];
    const std::size_t axes = device.state.axes.size();

    // Observe travel to learn each pedal's rest and full-travel positions.
    for (std::size_t a = 0; a < axes && a < pedalMin_.size(); ++a) {
        const double value = device.state.axes[a];
        pedalMin_[a] = std::min(pedalMin_[a], value);
        pedalMax_[a] = std::max(pedalMax_[a], value);
    }

    ImGui::TextWrapped("Press each pedal fully a few times so its travel is learned. The bars show the live value.");
    ImGui::Checkbox("Combined pedals (throttle and brake share one axis)", &combinedPedals_);
    sliderDouble("Pedal deadzone", pedalDeadzone_, 0.0, 0.3, "%.3f");
    ImGui::Separator();

    const auto applyPedal = [&](ControlRole role, int axis) {
        if (axis < 0 || axis >= static_cast<int>(axes)) {
            wheel.clearBinding(index, role);
            return;
        }
        wheel.bindAxis(index, role, axis, AxisHalf::Full);
        const double rest = axis < static_cast<int>(pedalRest_.size()) ? pedalRest_[axis] : 0.0;
        const double low = pedalMin_[axis];
        const double high = pedalMax_[axis];
        const double far = std::abs(rest - low) > std::abs(rest - high) ? low : high;
        input::AxisCalibration cal;
        cal.rawMin = std::min(rest, far);
        cal.rawMax = std::max(rest, far);
        cal.deadzone = pedalDeadzone_;
        cal.invert = rest > far;
        cal.useCenter = false;
        wheel.setCalibration(index, role, cal);
    };

    if (combinedPedals_) {
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("Combined axis", ("Axis " + std::to_string(combinedAxis_)).c_str())) {
            for (std::size_t a = 0; a < axes; ++a) {
                if (ImGui::Selectable(("Axis " + std::to_string(a)).c_str(), static_cast<int>(a) == combinedAxis_)) {
                    combinedAxis_ = static_cast<int>(a);
                }
            }
            ImGui::EndCombo();
        }
        wheel.bindAxis(index, ControlRole::Throttle, combinedAxis_, AxisHalf::PositiveHalf);
        wheel.bindAxis(index, ControlRole::Brake, combinedAxis_, AxisHalf::NegativeHalf);
        wheel.clearBinding(index, ControlRole::Clutch);
    } else {
        for (int p = 0; p < kPedalRoleCount; ++p) {
            const ControlRole role = kPedalRoles[p];
            const ControlBinding* binding = device.profile.find(role);
            int axis = binding != nullptr && binding->kind == BindingKind::Axis ? binding->axis.axis : -1;

            ImGui::PushID(p);
            ImGui::SetNextItemWidth(200.0f);
            if (ImGui::BeginCombo(input::toString(role),
                                  axis < 0 ? "unbound" : ("Axis " + std::to_string(axis)).c_str())) {
                if (ImGui::Selectable("unbound", axis < 0)) {
                    axis = -1;
                }
                for (std::size_t a = 0; a < axes; ++a) {
                    if (ImGui::Selectable(("Axis " + std::to_string(a)).c_str(), static_cast<int>(a) == axis)) {
                        axis = static_cast<int>(a);
                    }
                }
                ImGui::EndCombo();
            }
            if (axis >= 0 && axis < static_cast<int>(axes)) {
                ImGui::SameLine();
                ImGui::ProgressBar(static_cast<float>((device.state.axes[axis] + 1.0) * 0.5), ImVec2(120.0f, 0.0f), "");
            }
            applyPedal(role, axis);
            ImGui::PopID();
        }
    }

    ImGui::Separator();
    ImGui::Text("Throttle %.2f  Brake %.2f  Clutch %.2f", wheel.controls().throttle, wheel.controls().brake,
                wheel.controls().clutch);
}

void ControlSetupWizard::drawButtons(input::WheelManager& wheel) {
    if (wheel.devices().empty()) {
        ImGui::TextWrapped("No device to configure.");
        return;
    }
    const std::size_t index = clampDevice(wheel);
    const DeviceEntry& device = wheel.devices()[index];

    const int bound = pollListening(wheel);
    (void)bound;

    ImGui::TextWrapped("Click Assign, then press the button (or hat direction) you want. Everything can be skipped.");
    ImGui::Separator();

    if (ImGui::BeginTable("buttons", 3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Action");
        ImGui::TableSetupColumn("Binding");
        ImGui::TableSetupColumn("Assign");
        ImGui::TableHeadersRow();
        for (int r = 0; r < kButtonRoleCount; ++r) {
            const ControlRole role = kButtonRoles[r];
            const ControlBinding* binding = device.profile.find(role);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(input::toString(role));
            ImGui::TableSetColumnIndex(1);
            if (listeningRole_ == static_cast<int>(role)) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "press a control...");
            } else {
                ImGui::TextUnformatted(bindingLabel(binding).c_str());
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::PushID(r);
            if (ImGui::SmallButton("Assign")) {
                beginListening(wheel, static_cast<int>(role));
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear")) {
                wheel.clearBinding(index, role);
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    if (listeningRole_ >= 0) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Waiting for input for '%s'... (press Escape to cancel)",
                           input::toString(static_cast<ControlRole>(listeningRole_)));
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            listeningRole_ = -1;
        }
    }

    if (conflictRole_ >= 0) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "Button %d is already used by '%s'.", conflictButton_,
                           input::toString(static_cast<ControlRole>(conflictOther_)));
        if (ImGui::Button("Replace")) {
            wheel.clearBinding(index, static_cast<ControlRole>(conflictOther_));
            wheel.bindButton(index, static_cast<ControlRole>(conflictRole_), conflictButton_);
            conflictRole_ = -1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            conflictRole_ = -1;
        }
    }
}

void ControlSetupWizard::drawTest(input::WheelManager& wheel) {
    const input::Controls& controls = wheel.controls();
    ImGui::TextWrapped("Move every control to check it drives the right signal. Steering must rest near zero when "
                       "you let go.");
    ImGui::Separator();

    ImGui::Text("Steering");
    ImGui::SameLine(120.0f);
    ImGui::ProgressBar(static_cast<float>((controls.steer + 1.0) * 0.5), ImVec2(-1.0f, 0.0f), "");
    ImGui::Text("Throttle");
    ImGui::SameLine(120.0f);
    ImGui::ProgressBar(static_cast<float>(controls.throttle), ImVec2(-1.0f, 0.0f), "");
    ImGui::Text("Brake");
    ImGui::SameLine(120.0f);
    ImGui::ProgressBar(static_cast<float>(controls.brake), ImVec2(-1.0f, 0.0f), "");
    ImGui::Text("Clutch");
    ImGui::SameLine(120.0f);
    ImGui::ProgressBar(static_cast<float>(controls.clutch), ImVec2(-1.0f, 0.0f), "");
    ImGui::Text("Handbrake");
    ImGui::SameLine(120.0f);
    ImGui::ProgressBar(static_cast<float>(controls.handbrake), ImVec2(-1.0f, 0.0f), "");

    ImGui::Separator();
    ImGui::Text("Shift up: %s   Shift down: %s   Neutral: %s", controls.shiftUp ? "yes" : "-",
                controls.shiftDown ? "yes" : "-", controls.gearNeutral ? "yes" : "-");

    ImGui::Separator();
    const bool ffbAvailable = wheel.diagnostics().ffbAvailable;
    ImGui::BeginDisabled(!ffbAvailable);
    if (ImGui::Button("Test force feedback")) {
        wheel.ffb().startTest(1.0);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("%s", ffbAvailable ? "a short pulse will play" : "this device has no force feedback");
}

ControlSetupWizard::Result ControlSetupWizard::draw(input::WheelManager& wheel) {
    // Hot-plug: keep the device index valid and re-seed the current step when a
    // device is added or removed.
    if (static_cast<int>(wheel.devices().size()) != lastDeviceCount_) {
        lastDeviceCount_ = static_cast<int>(wheel.devices().size());
        lastInitStep_ = -1;
        listeningRole_ = -1;
        conflictRole_ = -1;
    }
    initStep(wheel);

    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(720.0f, 540.0f), ImGuiCond_Appearing);

    Result result = Result::None;
    bool open = true;
    if (ImGui::Begin("Control Setup", &open, ImGuiWindowFlags_NoCollapse)) {
        ImGui::Text("Step %d of %d", step_ + 1, kStepCount);
        ImGui::Separator();

        switch (step_) {
        case 0:
            drawDevices(wheel);
            break;
        case 1:
            drawCompatibility(wheel);
            break;
        case 2:
            drawSteering(wheel);
            break;
        case 3:
            drawPedals(wheel);
            break;
        case 4:
            drawButtons(wheel);
            break;
        default:
            drawTest(wheel);
            break;
        }

        ImGui::Separator();
        const bool hasDevice = !wheel.devices().empty();

        ImGui::BeginDisabled(step_ == 0);
        if (ImGui::Button("Back")) {
            --step_;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();

        if (step_ < kStepCount - 1) {
            if (ImGui::Button("Next")) {
                ++step_;
            }
        } else if (ImGui::Button(hasDevice ? "Finish" : "Finish (keyboard)")) {
            if (hasDevice) {
                wheel.storeProfiles();
            }
            result = Result::Finished;
        }

        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetWindowWidth() - 110.0f);
        if (ImGui::Button("Cancel")) {
            result = Result::Cancelled;
        }
    }
    ImGui::End();

    if (result != Result::None) {
        listeningRole_ = -1;
        conflictRole_ = -1;
    } else if (!open) {
        result = Result::Cancelled;
    }
    return result;
}

} // namespace sim::app
