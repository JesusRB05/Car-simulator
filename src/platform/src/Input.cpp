// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "platform/Input.hpp"

#include <algorithm>

#include <SDL3/SDL_keyboard.h>

#include "core/Log.hpp"

namespace sim::platform {

InputManager::InputManager()
{
    openAllDevices();
}

InputManager::~InputManager()
{
    for (auto& device : devices_)
    {
        if (device.joystick != nullptr)
        {
            SDL_CloseJoystick(device.joystick);
            device.joystick = nullptr;
        }
    }
}

void InputManager::openAllDevices()
{
    int count = 0;
    SDL_JoystickID* ids = SDL_GetJoysticks(&count);
    if (ids == nullptr)
    {
        return;
    }
    for (int i = 0; i < count; ++i)
    {
        openDevice(ids[i]);
    }
    SDL_free(ids);
}

void InputManager::openDevice(SDL_JoystickID instanceId)
{
    const auto alreadyOpen = std::any_of(devices_.begin(), devices_.end(),
                                         [instanceId](const InputDevice& d) { return d.instanceId == instanceId; });
    if (alreadyOpen)
    {
        return;
    }

    SDL_Joystick* joystick = SDL_OpenJoystick(instanceId);
    if (joystick == nullptr)
    {
        core::Log::get("input")->warn("failed to open joystick {}: {}", instanceId, SDL_GetError());
        return;
    }

    InputDevice device;
    device.instanceId = instanceId;
    device.joystick = joystick;
    const char* name = SDL_GetJoystickName(joystick);
    device.name = name != nullptr ? name : "Unknown device";
    device.isGamepad = SDL_IsGamepad(instanceId);

    const int axes = SDL_GetNumJoystickAxes(joystick);
    const int buttons = SDL_GetNumJoystickButtons(joystick);
    device.axes.assign(static_cast<std::size_t>(std::max(axes, 0)), 0.0F);
    device.buttons.assign(static_cast<std::size_t>(std::max(buttons, 0)), false);

    core::Log::get("input")->info("connected {} '{}' ({} axes, {} buttons)", device.isGamepad ? "gamepad" : "joystick",
                                  device.name, axes, buttons);
    devices_.push_back(std::move(device));
}

void InputManager::closeDevice(SDL_JoystickID instanceId)
{
    const auto it = std::find_if(devices_.begin(), devices_.end(),
                                 [instanceId](const InputDevice& d) { return d.instanceId == instanceId; });
    if (it == devices_.end())
    {
        return;
    }
    core::Log::get("input")->info("disconnected '{}'", it->name);
    if (it->joystick != nullptr)
    {
        SDL_CloseJoystick(it->joystick);
    }
    devices_.erase(it);
}

bool InputManager::processEvent(const SDL_Event& event)
{
    switch (event.type)
    {
    case SDL_EVENT_JOYSTICK_ADDED:
        openDevice(event.jdevice.which);
        return true;
    case SDL_EVENT_JOYSTICK_REMOVED:
        closeDevice(event.jdevice.which);
        return true;
    default:
        return false;
    }
}

void InputManager::refresh()
{
    for (auto& device : devices_)
    {
        if (device.joystick == nullptr)
        {
            continue;
        }
        for (std::size_t i = 0; i < device.axes.size(); ++i)
        {
            const Sint16 raw = SDL_GetJoystickAxis(device.joystick, static_cast<int>(i));
            device.axes[i] = raw < 0 ? static_cast<float>(raw) / 32768.0F : static_cast<float>(raw) / 32767.0F;
        }
        for (std::size_t i = 0; i < device.buttons.size(); ++i)
        {
            device.buttons[i] = SDL_GetJoystickButton(device.joystick, static_cast<int>(i));
        }
    }
}

bool InputManager::keyDown(SDL_Scancode scancode) const
{
    const bool* state = SDL_GetKeyboardState(nullptr);
    if (state == nullptr || scancode < 0)
    {
        return false;
    }
    return state[scancode];
}

std::vector<SDL_Scancode> InputManager::pressedKeys() const
{
    std::vector<SDL_Scancode> keys;
    const bool* state = SDL_GetKeyboardState(nullptr);
    if (state == nullptr)
    {
        return keys;
    }
    for (int i = 0; i < SDL_SCANCODE_COUNT; ++i)
    {
        if (state[i])
        {
            keys.push_back(static_cast<SDL_Scancode>(i));
        }
    }
    return keys;
}

} // namespace sim::platform
