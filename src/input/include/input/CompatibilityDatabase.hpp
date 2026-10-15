// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <vector>

#include "input/Types.hpp"

namespace sim::input {

// One verified-or-reported compatibility entry. Capabilities here are *declared*
// (a baseline); the live counts always come from the device itself, so a wheel
// that reports more axes than the entry lists still works.
//
// A `productId` of 0 means "any product of this vendor whose name matches
// `nameMatch`": those entries are only used when the exact VID/PID lookup
// misses, and only when the reported name contains the hint.
struct DeviceModel {
    std::uint16_t vendorId = 0;
    std::uint16_t productId = 0;
    const char* nameMatch = nullptr; // case-insensitive substring, or null
    const char* manufacturer = "";
    const char* model = "";
    DeviceKind kind = DeviceKind::Unknown;
    FfbCapabilities ffb{};
    bool requiresVendorDriver = false;
    VerificationStatus status = VerificationStatus::Unknown;
    const char* source = "";
    const char* notes = ""; // known limitations, or empty
};

// Result of asking the database about one device. When `model` is null the
// device was not in the database and the other fields hold a best-effort guess
// derived from the vendor id and the product name.
struct Identification {
    const DeviceModel* model = nullptr;
    DeviceKind kind = DeviceKind::Unknown;
    std::string manufacturer;
    std::string modelName;
    VerificationStatus status = VerificationStatus::Unknown;
    std::string source;
    std::string notes;
    FfbCapabilities declaredFfb{};
    bool requiresVendorDriver = false;

    bool known() const { return model != nullptr; }
};

// Read-only registry of known wheels, pedals, shifters and handbrakes. The
// table is deliberately conservative: everything is marked with the tier of
// evidence behind it, and unknown devices are simply classified as generic.
class CompatibilityDatabase {
public:
    static CompatibilityDatabase& instance();

    // Identifies a device. Order: exact VID/PID, then vendor + name hint, then
    // vendor/name heuristics, finally a plain generic classification.
    Identification identify(const DeviceDescriptor& device) const;

    // Every entry, for the diagnostics / compatibility panel.
    const std::vector<DeviceModel>& entries() const { return entries_; }

    // Human-readable vendor for a USB vendor id, or "" when unknown.
    static const char* vendorName(std::uint16_t vendorId);

private:
    CompatibilityDatabase();
    Identification classify(const DeviceDescriptor& device) const;

    std::vector<DeviceModel> entries_;
};

} // namespace sim::input
