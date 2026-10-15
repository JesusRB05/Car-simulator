// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "platform/Input.hpp"

#include <algorithm>
#include <cmath>

#include <SDL3/SDL_keyboard.h>

#include "core/Log.hpp"

namespace sim::platform {

namespace {

sim::input::FfbCapabilities detectFfb(SDL_Joystick* joystick) {
    sim::input::FfbCapabilities caps;
    const SDL_PropertiesID props = SDL_GetJoystickProperties(joystick);
    if (props == 0) {
        return caps;
    }
    caps.rumble = SDL_GetBooleanProperty(props, SDL_PROP_JOYSTICK_CAP_RUMBLE_BOOLEAN, false);
    caps.triggerRumble = SDL_GetBooleanProperty(props, SDL_PROP_JOYSTICK_CAP_TRIGGER_RUMBLE_BOOLEAN, false);
    caps.led = SDL_GetBooleanProperty(props, SDL_PROP_JOYSTICK_CAP_RGB_LED_BOOLEAN, false) ||
               SDL_GetBooleanProperty(props, SDL_PROP_JOYSTICK_CAP_MONO_LED_BOOLEAN, false);
    return caps;
}

sim::input::DeviceKind kindFromType(SDL_JoystickType type) {
    switch (type) {
    case SDL_JOYSTICK_TYPE_WHEEL:
        return sim::input::DeviceKind::WheelBase;
    case SDL_JOYSTICK_TYPE_GAMEPAD:
        return sim::input::DeviceKind::Gamepad;
    case SDL_JOYSTICK_TYPE_THROTTLE:
        return sim::input::DeviceKind::GenericHid;
    default:
        return sim::input::DeviceKind::Unknown;
    }
}

sim::input::DeviceBus busFromConnection(SDL_JoystickConnectionState state) {
    switch (state) {
    case SDL_JOYSTICK_CONNECTION_WIRELESS:
        return sim::input::DeviceBus::Bluetooth;
    case SDL_JOYSTICK_CONNECTION_WIRED:
        return sim::input::DeviceBus::Usb;
    default:
        return sim::input::DeviceBus::Unknown;
    }
}

} // namespace

// --- SdlFfbOutput ---------------------------------------------------------

void SdlFfbOutput::setRumble(double low, double high, int durationMs) {
    if (joystick_ == nullptr) {
        return;
    }
    const double clampedLow = std::clamp(low, 0.0, 1.0);
    const double clampedHigh = std::clamp(high, 0.0, 1.0);
    const Uint32 duration = durationMs < 0 ? 0U : static_cast<Uint32>(durationMs);
    if (caps_.rumble) {
        SDL_RumbleJoystick(joystick_, static_cast<Uint16>(clampedLow * 65535.0),
                           static_cast<Uint16>(clampedHigh * 65535.0), duration);
    }
    if (caps_.triggerRumble) {
        SDL_RumbleJoystickTriggers(joystick_, static_cast<Uint16>(clampedLow * 65535.0),
                                   static_cast<Uint16>(clampedHigh * 65535.0), duration);
    }
}

void SdlFfbOutput::setConstantForce(double) {
    // Not exposed by SDL's portable force feedback API; see the class comment.
}

void SdlFfbOutput::setCondition(sim::input::FfbCondition, double) {
    // Same as above: only reachable through the Windows-only raw effect path.
}

void SdlFfbOutput::setLed(double red, double green, double blue) {
    if (joystick_ == nullptr || !caps_.led) {
        return;
    }
    const auto channel = [](double value) { return static_cast<Uint8>(std::clamp(value, 0.0, 1.0) * 255.0); };
    SDL_SetJoystickLED(joystick_, channel(red), channel(green), channel(blue));
}

void SdlFfbOutput::stopAll() {
    if (joystick_ == nullptr) {
        return;
    }
    if (caps_.rumble) {
        SDL_RumbleJoystick(joystick_, 0, 0, 0);
    }
    if (caps_.triggerRumble) {
        SDL_RumbleJoystickTriggers(joystick_, 0, 0, 0);
    }
}

// --- InputManager ---------------------------------------------------------

InputManager::InputManager() {
    openAllDevices();
}

InputManager::~InputManager() {
    for (InputDevice& device : devices_) {
        if (device.ffb) {
            device.ffb->stopAll();
        }
        if (device.joystick != nullptr) {
            SDL_CloseJoystick(device.joystick);
            device.joystick = nullptr;
        }
    }
}

void InputManager::openAllDevices() {
    int count = 0;
    SDL_JoystickID* ids = SDL_GetJoysticks(&count);
    if (ids == nullptr) {
        return;
    }
    for (int i = 0; i < count; ++i) {
        openDevice(ids[i]);
    }
    SDL_free(ids);
}

void InputManager::openDevice(SDL_JoystickID instanceId) {
    if (findDevice(static_cast<std::uint32_t>(instanceId)) != nullptr) {
        return;
    }

    SDL_Joystick* joystick = SDL_OpenJoystick(instanceId);
    if (joystick == nullptr) {
        core::Log::get("input")->warn("failed to open joystick {}: {}", instanceId, SDL_GetError());
        return;
    }

    InputDevice device;
    device.instanceId = instanceId;
    device.joystick = joystick;
    const char* name = SDL_GetJoystickName(joystick);
    device.name = name != nullptr ? name : "Unknown device";
    const char* serial = SDL_GetJoystickSerial(joystick);
    if (serial != nullptr) {
        device.serial = serial;
    }
    const char* path = SDL_GetJoystickPath(joystick);
    if (path != nullptr) {
        device.path = path;
    }
    device.vendorId = SDL_GetJoystickVendor(joystick);
    device.productId = SDL_GetJoystickProduct(joystick);
    device.productVersion = SDL_GetJoystickProductVersion(joystick);
    device.reportedKind = kindFromType(SDL_GetJoystickType(joystick));
    device.bus = busFromConnection(SDL_GetJoystickConnectionState(joystick));
    device.isGamepad = SDL_IsGamepad(instanceId);

    const int axes = SDL_GetNumJoystickAxes(joystick);
    const int buttons = SDL_GetNumJoystickButtons(joystick);
    const int hats = SDL_GetNumJoystickHats(joystick);
    device.axes.assign(static_cast<std::size_t>(std::max(axes, 0)), 0.0F);
    device.buttons.assign(static_cast<std::size_t>(std::max(buttons, 0)), false);
    device.hats.assign(static_cast<std::size_t>(std::max(hats, 0)), 0);

    const sim::input::FfbCapabilities caps = detectFfb(joystick);
    if (caps.any()) {
        device.ffb = std::make_shared<SdlFfbOutput>(joystick, caps);
    }

    core::Log::get("input")->info("connected {} '{}' (VID {:04x} PID {:04x}, {} axes, {} buttons, {} hats{})",
                                  device.isGamepad ? "gamepad" : "joystick", device.name, device.vendorId,
                                  device.productId, axes, buttons, hats, caps.any() ? ", FFB" : "");
    devices_.push_back(std::move(device));
}

void InputManager::closeDevice(SDL_JoystickID instanceId) {
    const auto it = std::find_if(devices_.begin(), devices_.end(),
                                 [instanceId](const InputDevice& d) { return d.instanceId == instanceId; });
    if (it == devices_.end()) {
        return;
    }
    core::Log::get("input")->info("disconnected '{}'", it->name);
    if (it->ffb) {
        it->ffb->stopAll();
    }
    if (it->joystick != nullptr) {
        SDL_CloseJoystick(it->joystick);
    }
    devices_.erase(it);
}

bool InputManager::processEvent(const SDL_Event& event) {
    switch (event.type) {
    case SDL_EVENT_JOYSTICK_ADDED:
        openDevice(event.jdevice.which);
        return true;
    case SDL_EVENT_JOYSTICK_REMOVED:
        closeDevice(event.jdevice.which);
        return true;
    case SDL_EVENT_WINDOW_FOCUS_GAINED:
        focused_ = true;
        return false;
    case SDL_EVENT_WINDOW_FOCUS_LOST:
        focused_ = false;
        return false;
    default:
        return false;
    }
}

void InputManager::refresh() {
    for (InputDevice& device : devices_) {
        if (device.joystick == nullptr) {
            continue;
        }
        for (std::size_t i = 0; i < device.axes.size(); ++i) {
            const Sint16 raw = SDL_GetJoystickAxis(device.joystick, static_cast<int>(i));
            device.axes[i] = raw < 0 ? static_cast<float>(raw) / 32768.0F : static_cast<float>(raw) / 32767.0F;
        }
        for (std::size_t i = 0; i < device.buttons.size(); ++i) {
            device.buttons[i] = SDL_GetJoystickButton(device.joystick, static_cast<int>(i));
        }
        for (std::size_t i = 0; i < device.hats.size(); ++i) {
            device.hats[i] = SDL_GetJoystickHat(device.joystick, static_cast<int>(i));
        }
    }
}

bool InputManager::keyDown(SDL_Scancode scancode) const {
    const bool* state = SDL_GetKeyboardState(nullptr);
    if (state == nullptr || scancode < 0) {
        return false;
    }
    return state[scancode];
}

std::vector<SDL_Scancode> InputManager::pressedKeys() const {
    std::vector<SDL_Scancode> keys;
    const bool* state = SDL_GetKeyboardState(nullptr);
    if (state == nullptr) {
        return keys;
    }
    for (int i = 0; i < SDL_SCANCODE_COUNT; ++i) {
        if (state[i]) {
            keys.push_back(static_cast<SDL_Scancode>(i));
        }
    }
    return keys;
}

// --- IInputBackend --------------------------------------------------------

void InputManager::poll() {
    refresh();
}

std::vector<sim::input::DeviceDescriptor> InputManager::deviceDescriptors() const {
    std::vector<sim::input::DeviceDescriptor> descriptors;
    descriptors.reserve(devices_.size());
    for (const InputDevice& device : devices_) {
        sim::input::DeviceDescriptor descriptor;
        descriptor.backendId = std::to_string(device.instanceId);
        descriptor.instanceId = static_cast<std::uint32_t>(device.instanceId);
        descriptor.vendorId = device.vendorId;
        descriptor.productId = device.productId;
        descriptor.productVersion = device.productVersion;
        descriptor.bus = device.bus;
        descriptor.isGamepad = device.isGamepad;
        descriptor.name = device.name;
        descriptor.serial = device.serial;
        descriptor.path = device.path;
        descriptor.caps.axes = static_cast<int>(device.axes.size());
        descriptor.caps.buttons = static_cast<int>(device.buttons.size());
        descriptor.caps.hats = static_cast<int>(device.hats.size());
        if (device.ffb) {
            descriptor.caps.ffb = device.ffb->capabilities();
        }
        descriptors.push_back(std::move(descriptor));
    }
    return descriptors;
}

bool InputManager::readState(std::uint32_t instanceId, sim::input::RawDeviceState& out) const {
    const InputDevice* device = findDevice(instanceId);
    if (device == nullptr) {
        return false;
    }
    out.axes.assign(device->axes.begin(), device->axes.end());
    out.buttons = device->buttons;
    out.hats = device->hats;
    return true;
}

sim::input::IFfbOutput* InputManager::ffbOutput(std::uint32_t instanceId) {
    InputDevice* device = findDevice(instanceId);
    return device != nullptr && device->ffb ? device->ffb.get() : nullptr;
}

InputDevice* InputManager::findDevice(std::uint32_t instanceId) {
    const auto it = std::find_if(devices_.begin(), devices_.end(), [instanceId](const InputDevice& d) {
        return static_cast<std::uint32_t>(d.instanceId) == instanceId;
    });
    return it == devices_.end() ? nullptr : &*it;
}

const InputDevice* InputManager::findDevice(std::uint32_t instanceId) const {
    const auto it = std::find_if(devices_.begin(), devices_.end(), [instanceId](const InputDevice& d) {
        return static_cast<std::uint32_t>(d.instanceId) == instanceId;
    });
    return it == devices_.end() ? nullptr : &*it;
}

} // namespace sim::platform
