// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "input/Types.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace sim::input {

const char* toString(DeviceKind kind) {
    switch (kind) {
    case DeviceKind::WheelBase:
        return "wheel base";
    case DeviceKind::Pedals:
        return "pedals";
    case DeviceKind::Shifter:
        return "shifter";
    case DeviceKind::Handbrake:
        return "handbrake";
    case DeviceKind::Gamepad:
        return "gamepad";
    case DeviceKind::GenericHid:
        return "generic HID";
    case DeviceKind::Unknown:
        return "unknown";
    }
    return "unknown";
}

const char* toString(DeviceBus bus) {
    switch (bus) {
    case DeviceBus::Usb:
        return "USB";
    case DeviceBus::Bluetooth:
        return "Bluetooth";
    case DeviceBus::Virtual:
        return "virtual";
    case DeviceBus::Network:
        return "network";
    case DeviceBus::Unknown:
        return "unknown";
    }
    return "unknown";
}

const char* toString(VerificationStatus status) {
    switch (status) {
    case VerificationStatus::VerifiedHardware:
        return "verified (hardware)";
    case VerificationStatus::VerifiedDocumentation:
        return "verified (documentation)";
    case VerificationStatus::ManufacturerDeclared:
        return "manufacturer declared";
    case VerificationStatus::CommunityReported:
        return "community reported";
    case VerificationStatus::Unknown:
        return "unknown";
    }
    return "unknown";
}

std::string deviceKey(const DeviceDescriptor& device) {
    if (device.vendorId != 0 && device.productId != 0) {
        char buffer[16];
        std::snprintf(buffer, sizeof(buffer), "%04x:%04x", device.vendorId, device.productId);
        return buffer;
    }
    // No USB ids: fall back to a sanitised product name so DIY boards still get
    // a stable key (they usually share the same name across reconnects).
    std::string key = device.name;
    for (char& c : key) {
        if (!std::isalnum(static_cast<unsigned char>(c))) {
            c = '_';
        }
    }
    if (key.empty()) {
        key = "unknown";
    }
    return key;
}

} // namespace sim::input
