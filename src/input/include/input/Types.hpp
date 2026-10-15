// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <string>

namespace sim::input {

// What a device is, as far as it can be told apart. The wheel bases, the
// standalone pedal sets, the shifters and the DIY boards all show up as USB
// HID devices; the kind is a best-effort classification, never a promise.
enum class DeviceKind {
    Unknown,
    WheelBase,  // steering wheel base, usually with FFB
    Pedals,     // standalone pedal set
    Shifter,    // sequential or H-pattern shifter
    Handbrake,  // analog handbrake
    Gamepad,    // standard pad (XInput-style or SDL-recognised)
    GenericHid, // anything else with axes / buttons
};

// How the device reaches the machine. Used for diagnostics and to warn about
// vendor software requirements; it never changes how the device is read.
enum class DeviceBus {
    Unknown,
    Usb,
    Bluetooth,
    Virtual, // software / emulated device (vJoy, ViGEm, Steam Input...)
    Network, // remote / streaming
};

// How sure we are about a compatibility entry. Mirrors the four tiers of the
// project's compatibility policy; never promote an entry without evidence.
enum class VerificationStatus {
    Unknown = 0,           // no data, treated as a generic device
    CommunityReported = 1, // reported by users, not verified here
    ManufacturerDeclared = 2,
    VerifiedDocumentation = 3,
    VerifiedHardware = 4, // exercised on real hardware by a maintainer
};

const char* toString(DeviceKind kind);
const char* toString(DeviceBus bus);
const char* toString(VerificationStatus status);

// Force feedback capabilities. A device may read axes and buttons perfectly and
// still support none of these; input support and FFB support are independent.
struct FfbCapabilities {
    bool constantForce = false; // directional torque (the main wheel effect)
    bool spring = false;        // centring spring
    bool damper = false;        // velocity-dependent resistance
    bool friction = false;      // static resistance
    bool rumble = false;        // whole-body vibration motors
    bool triggerRumble = false;
    bool led = false; // addressable LED / rev lights

    bool any() const { return constantForce || spring || damper || friction || rumble || triggerRumble || led; }
    bool anyForce() const { return constantForce || spring || damper || friction; }
};

// Runtime capability snapshot read from the backend. The counts are the real
// numbers reported by the device, not the ones declared by the database.
struct DeviceCapabilities {
    int axes = 0;
    int buttons = 0;
    int hats = 0;
    FfbCapabilities ffb{};
    bool requiresVendorDriver = false; // device needs vendor software to work fully
};

// Everything the backend knows about one connected device. `backendId` is a
// stable, backend-specific identifier (the SDL instance id on SDL3); it can be
// reused after a reconnect, so it is never used for identity in profiles.
struct DeviceDescriptor {
    std::string backendId;
    std::uint32_t instanceId = 0;
    std::uint16_t vendorId = 0;
    std::uint16_t productId = 0;
    std::uint16_t productVersion = 0;
    DeviceBus bus = DeviceBus::Unknown;
    bool isGamepad = false;
    std::string name;
    std::string manufacturer; // filled from the database when known, else the OS name
    std::string serial;
    std::string path;
    DeviceCapabilities caps{};
};

// A stable "VID:PID" key used to look profiles up across reconnects. Devices
// with no USB ids (virtual boards) fall back to the sanitised product name.
std::string deviceKey(const DeviceDescriptor& device);

} // namespace sim::input
