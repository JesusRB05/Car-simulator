// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <SDL3/SDL.h>

namespace sim::platform {

// Live state of one input device (keyboard is handled separately). Wheels,
// pedals and gamepads all appear here as SDL joysticks; `isGamepad` marks the
// ones SDL recognises as standard gamepads.
struct InputDevice {
    SDL_JoystickID instanceId = 0;
    SDL_Joystick* joystick = nullptr;
    std::string name;
    bool isGamepad = false;
    std::vector<float> axes; // normalised to [-1, 1]
    std::vector<bool> buttons;
};

// Tracks keyboard state and enumerates / hot-plugs joysticks, wheels and
// gamepads. The ImGui input panel renders directly from this.
class InputManager {
public:
    InputManager();
    ~InputManager();

    InputManager(const InputManager&) = delete;
    InputManager& operator=(const InputManager&) = delete;

    // Consumes SDL events (hot-plug). Returns true if the event was relevant.
    bool processEvent(const SDL_Event& event);

    // Re-polls axis and button values. Call once per frame.
    void refresh();

    bool keyDown(SDL_Scancode scancode) const;
    std::vector<SDL_Scancode> pressedKeys() const;

    const std::vector<InputDevice>& devices() const { return devices_; }
    std::size_t joystickCount() const { return devices_.size(); }

private:
    void openAllDevices();
    void openDevice(SDL_JoystickID instanceId);
    void closeDevice(SDL_JoystickID instanceId);

    std::vector<InputDevice> devices_;
};

} // namespace sim::platform
