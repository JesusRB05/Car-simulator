// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

#include "input/ControlMapping.hpp"
#include "input/ForceFeedback.hpp"

namespace sim::core {
class Config;
} // namespace sim::core

namespace sim::input {

// Persists device profiles and the global force feedback settings in a TOML
// document (the application's user config). Profiles are keyed by the stable
// device key, so a wheel keeps its mapping across reconnects and across
// different USB ports, and several devices can be connected at once.
//
// The store works entirely in memory; the caller decides when to flush to disk,
// which keeps the tests off the filesystem.
class ProfileStore {
public:
    static constexpr int kCurrentVersion = 1;

    explicit ProfileStore(core::Config& config) : config_(config) {}

    // Returns the stored profile for `key`, or a copy of `fallback` when there
    // is none. Migrates older versions forward on the fly.
    DeviceProfile loadProfile(const std::string& key, const DeviceProfile& fallback) const;

    // Stores (or replaces) a profile, stamping the current version.
    void storeProfile(const DeviceProfile& profile);

    void removeProfile(const std::string& key);
    std::vector<std::string> profileKeys() const;

    FfbSettings loadFfbSettings() const;
    void storeFfbSettings(const FfbSettings& settings);

    // Turns a device key into a TOML-safe key (bare keys only allow
    // A-Za-z0-9_-), so "046d:c29b" becomes "046d_c29b".
    static std::string sanitizeKey(const std::string& key);

private:
    core::Config& config_;
};

} // namespace sim::input
