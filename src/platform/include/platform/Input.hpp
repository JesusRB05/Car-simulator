// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <SDL3/SDL.h>

#include "input/Backend.hpp"
#include "input/Types.hpp"

namespace sim::platform {

// Force feedback sink for one SDL joystick. SDL exposes rumble (both the whole
// body and the triggers) and, where the device supports it, the addressable
// LED. Constant force and the spring/damper/friction conditions are part of the
// DirectInput force feedback model: SDL only forwards them, on Windows, through
// the raw SDL_SendJoystickEffect path, so they are reported as unsupported here
// and documented as a follow-up rather than faked.
class SdlFfbOutput final : public sim::input::IFfbOutput {
public:
    SdlFfbOutput(SDL_Joystick* joystick, sim::input::FfbCapabilities caps) : joystick_(joystick), caps_(caps) {}

    sim::input::FfbCapabilities capabilities() const override { return caps_; }
    void setRumble(double low, double high, int durationMs) override;
    void setConstantForce(double magnitude) override;
    void setCondition(sim::input::FfbCondition condition, double coefficient) override;
    void setLed(double red, double green, double blue) override;
    void stopAll() override;

private:
    SDL_Joystick* joystick_ = nullptr;
    sim::input::FfbCapabilities caps_{};
};

// Live state of one input device (keyboard is handled separately). Wheels,
// pedals and gamepads all appear here as SDL joysticks; `isGamepad` marks the
// ones SDL recognises as standard gamepads.
struct InputDevice {
    SDL_JoystickID instanceId = 0;
    SDL_Joystick* joystick = nullptr;
    std::string name;
    std::string serial;
    std::string path;
    std::uint16_t vendorId = 0;
    std::uint16_t productId = 0;
    std::uint16_t productVersion = 0;
    sim::input::DeviceKind reportedKind = sim::input::DeviceKind::Unknown;
    sim::input::DeviceBus bus = sim::input::DeviceBus::Unknown;
    bool isGamepad = false;
    std::vector<float> axes; // normalised to [-1, 1]
    std::vector<bool> buttons;
    std::vector<int> hats; // SDL_HAT_* bitmask
    std::shared_ptr<SdlFfbOutput> ffb;
};

// Tracks keyboard state and enumerates / hot-plugs joysticks, wheels and
// gamepads. It owns every SDL_Joystick, so it also implements the render-free
// sim::input backend: the WheelManager is fed from here, and the ImGui input
// panel renders directly from this.
class InputManager final : public sim::input::IInputBackend {
public:
    InputManager();
    ~InputManager() override;

    InputManager(const InputManager&) = delete;
    InputManager& operator=(const InputManager&) = delete;

    // Consumes SDL events (hot-plug, focus). Returns true if the event was
    // relevant.
    bool processEvent(const SDL_Event& event);

    // Re-polls axis, button and hat values. Call once per frame.
    void refresh();

    bool keyDown(SDL_Scancode scancode) const;
    std::vector<SDL_Scancode> pressedKeys() const;

    const std::vector<InputDevice>& devices() const { return devices_; }
    std::size_t joystickCount() const { return devices_.size(); }

    void setWindowFocused(bool focused) { focused_ = focused; }

    // --- sim::input::IInputBackend ---
    void poll() override;
    std::vector<sim::input::DeviceDescriptor> deviceDescriptors() const override;
    bool readState(std::uint32_t instanceId, sim::input::RawDeviceState& out) const override;
    bool windowFocused() const override { return focused_; }
    sim::input::IFfbOutput* ffbOutput(std::uint32_t instanceId) override;

private:
    void openAllDevices();
    void openDevice(SDL_JoystickID instanceId);
    void closeDevice(SDL_JoystickID instanceId);
    InputDevice* findDevice(std::uint32_t instanceId);
    const InputDevice* findDevice(std::uint32_t instanceId) const;

    std::vector<InputDevice> devices_;
    bool focused_ = true;
};

} // namespace sim::platform
