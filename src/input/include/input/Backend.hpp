// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <vector>

#include "input/Types.hpp"

namespace sim::input {

// Raw state of one device for one frame. Axes are normalised to [-1, 1] where
// the backend can (SDL reports signed 16-bit axes); hats use the SDL_HAT_*
// bitmask (1 = up, 2 = right, 4 = down, 8 = left, 0 = centred).
struct RawDeviceState {
    std::vector<double> axes;
    std::vector<bool> buttons;
    std::vector<int> hats;
};

// A condition (resistance) effect offered by the DirectInput force feedback
// model. Not every wheel supports every condition.
enum class FfbCondition {
    Spring,   // restores the wheel towards centre
    Damper,   // resists movement, proportional to speed
    Friction, // constant resistance while moving
};

// One device's force feedback sink. Implemented by the SDL backend on top of
// SDL_RumbleJoystick / SDL_SendJoystickEffect. A backend returns an output for
// devices that expose any FFB; the manager never assumes an effect is present.
class IFfbOutput {
public:
    virtual ~IFfbOutput() = default;

    virtual FfbCapabilities capabilities() const = 0;

    // Whole-body rumble, both channels in [0, 1]. `durationMs` of 0 keeps the
    // effect until the next call.
    virtual void setRumble(double low, double high, int durationMs) = 0;

    // Directional steering torque in [-1, 1] (negative = counter-clockwise).
    // Implemented through the DirectInput effect API where available.
    virtual void setConstantForce(double magnitude) = 0;

    // Condition coefficient in [-1, 1] (0 = off, negative = inverted).
    virtual void setCondition(FfbCondition condition, double coefficient) = 0;

    // Rev lights / RGB LED, channels in [0, 1]. No-op when unsupported.
    virtual void setLed(double red, double green, double blue) = 0;

    // Cancels every running effect. Must be safe to call at any time, including
    // right before the device is closed.
    virtual void stopAll() = 0;
};

// The operating-system input layer. SDL3 implements this on every desktop
// platform; the tests implement it with a mock, so the whole manager runs
// without hardware. The backend owns device discovery and hot-plug; it must be
// polled exactly once per frame before the devices or states are read.
class IInputBackend {
public:
    virtual ~IInputBackend() = default;

    // Refreshes the device list and every device's state.
    virtual void poll() = 0;

    // Snapshot of the currently connected devices.
    virtual std::vector<DeviceDescriptor> deviceDescriptors() const = 0;

    // Copies the latest raw state of `instanceId` into `out`. Returns false when
    // the device is gone.
    virtual bool readState(std::uint32_t instanceId, RawDeviceState& out) const = 0;

    // Whether the application window is focused. Force feedback is stopped while
    // it is not, so a background window never keeps the wheel loaded.
    virtual bool windowFocused() const = 0;

    // Force feedback sink for a device, or nullptr when it has none.
    virtual IFfbOutput* ffbOutput(std::uint32_t instanceId) = 0;
};

} // namespace sim::input
