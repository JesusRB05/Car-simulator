// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "input/Profiles.hpp"

#include <algorithm>
#include <cctype>

#include "core/Config.hpp"

namespace sim::input {

namespace {

const char* halfToString(AxisHalf half) {
    switch (half) {
    case AxisHalf::NegativeHalf:
        return "negative";
    case AxisHalf::PositiveHalf:
        return "positive";
    case AxisHalf::Full:
        break;
    }
    return "full";
}

AxisHalf halfFromString(const std::string& value) {
    if (value == "negative") {
        return AxisHalf::NegativeHalf;
    }
    if (value == "positive") {
        return AxisHalf::PositiveHalf;
    }
    return AxisHalf::Full;
}

const char* kindToString(BindingKind kind) {
    switch (kind) {
    case BindingKind::Axis:
        return "axis";
    case BindingKind::Button:
        return "button";
    case BindingKind::Hat:
        return "hat";
    case BindingKind::None:
        break;
    }
    return "none";
}

BindingKind kindFromString(const std::string& value) {
    if (value == "axis") {
        return BindingKind::Axis;
    }
    if (value == "button") {
        return BindingKind::Button;
    }
    if (value == "hat") {
        return BindingKind::Hat;
    }
    return BindingKind::None;
}

std::string roleKey(const std::string& base, int role, const char* field) {
    return base + ".roles." + std::to_string(role) + "." + field;
}

} // namespace

std::string ProfileStore::sanitizeKey(const std::string& key) {
    std::string out = key;
    for (char& c : out) {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) {
            c = '_';
        }
    }
    if (out.empty()) {
        out = "unknown";
    }
    return out;
}

DeviceProfile ProfileStore::loadProfile(const std::string& key, const DeviceProfile& fallback) const {
    const std::string base = "input.profiles." + sanitizeKey(key);
    if (!config_.contains(base + ".version")) {
        DeviceProfile copy = fallback;
        copy.key = key;
        return copy;
    }

    DeviceProfile profile;
    profile.key = key;
    profile.version = static_cast<int>(config_.getInt(base + ".version", kCurrentVersion));
    profile.modelName = config_.getString(base + ".model", fallback.modelName);
    profile.steeringRangeDegrees = config_.getDouble(base + ".steeringRange", fallback.steeringRangeDegrees);

    for (int r = 0; r < static_cast<int>(ControlRole::Count); ++r) {
        const std::string kindKey = roleKey(base, r, "kind");
        if (!config_.contains(kindKey)) {
            continue;
        }
        const BindingKind kind = kindFromString(config_.getString(kindKey, "none"));
        const ControlRole role = static_cast<ControlRole>(r);
        ControlBinding binding;
        binding.kind = kind;
        binding.axis.axis = static_cast<int>(config_.getInt(roleKey(base, r, "axis"), -1));
        binding.axis.half = halfFromString(config_.getString(roleKey(base, r, "half"), "full"));
        binding.axis.calibration.rawMin = config_.getDouble(roleKey(base, r, "rawMin"), -1.0);
        binding.axis.calibration.rawMax = config_.getDouble(roleKey(base, r, "rawMax"), 1.0);
        binding.axis.calibration.center = config_.getDouble(roleKey(base, r, "center"), 0.0);
        binding.axis.calibration.deadzone = config_.getDouble(roleKey(base, r, "deadzone"), 0.0);
        binding.axis.calibration.saturation = config_.getDouble(roleKey(base, r, "saturation"), 0.0);
        binding.axis.calibration.invert = config_.getBool(roleKey(base, r, "invert"), false);
        binding.axis.calibration.useCenter = config_.getBool(roleKey(base, r, "useCenter"), true);
        binding.axis.calibration.active = config_.getBool(roleKey(base, r, "active"), true);
        binding.button = static_cast<int>(config_.getInt(roleKey(base, r, "button"), -1));
        binding.hat = static_cast<int>(config_.getInt(roleKey(base, r, "hat"), -1));
        binding.hatDirection = static_cast<int>(config_.getInt(roleKey(base, r, "hatDir"), 0));
        profile.bindings[role] = binding;
    }

    // Fill any role the stored profile does not define from the fallback, so a
    // profile saved by an older build still works.
    for (const auto& [role, binding] : fallback.bindings) {
        if (profile.bindings.find(role) == profile.bindings.end()) {
            profile.bindings[role] = binding;
        }
    }
    return profile;
}

void ProfileStore::storeProfile(const DeviceProfile& profile) {
    const std::string base = "input.profiles." + sanitizeKey(profile.key);
    config_.setInt(base + ".version", kCurrentVersion);
    config_.setString(base + ".model", profile.modelName);
    config_.setDouble(base + ".steeringRange", profile.steeringRangeDegrees);

    // Clear roles that are no longer present, then write the current ones.
    for (int r = 0; r < static_cast<int>(ControlRole::Count); ++r) {
        config_.erase(roleKey(base, r, "kind"));
    }
    for (const auto& [role, binding] : profile.bindings) {
        const int r = static_cast<int>(role);
        config_.setString(roleKey(base, r, "kind"), kindToString(binding.kind));
        config_.setInt(roleKey(base, r, "axis"), binding.axis.axis);
        config_.setString(roleKey(base, r, "half"), halfToString(binding.axis.half));
        config_.setDouble(roleKey(base, r, "rawMin"), binding.axis.calibration.rawMin);
        config_.setDouble(roleKey(base, r, "rawMax"), binding.axis.calibration.rawMax);
        config_.setDouble(roleKey(base, r, "center"), binding.axis.calibration.center);
        config_.setDouble(roleKey(base, r, "deadzone"), binding.axis.calibration.deadzone);
        config_.setDouble(roleKey(base, r, "saturation"), binding.axis.calibration.saturation);
        config_.setBool(roleKey(base, r, "invert"), binding.axis.calibration.invert);
        config_.setBool(roleKey(base, r, "useCenter"), binding.axis.calibration.useCenter);
        config_.setBool(roleKey(base, r, "active"), binding.axis.calibration.active);
        config_.setInt(roleKey(base, r, "button"), binding.button);
        config_.setInt(roleKey(base, r, "hat"), binding.hat);
        config_.setInt(roleKey(base, r, "hatDir"), binding.hatDirection);
    }
}

void ProfileStore::removeProfile(const std::string& key) {
    const std::string base = "input.profiles." + sanitizeKey(key);
    config_.erase(base);
}

std::vector<std::string> ProfileStore::profileKeys() const {
    // Keys are only needed for display; the manager keeps the authoritative
    // list of connected devices, so an empty result is acceptable.
    return {};
}

FfbSettings ProfileStore::loadFfbSettings() const {
    FfbSettings settings;
    settings.enabled = config_.getBool("input.ffb.enabled", settings.enabled);
    settings.masterGain = config_.getDouble("input.ffb.masterGain", settings.masterGain);
    settings.constantForceGain = config_.getDouble("input.ffb.constantGain", settings.constantForceGain);
    settings.springGain = config_.getDouble("input.ffb.springGain", settings.springGain);
    settings.damperGain = config_.getDouble("input.ffb.damperGain", settings.damperGain);
    settings.frictionGain = config_.getDouble("input.ffb.frictionGain", settings.frictionGain);
    settings.rumbleGain = config_.getDouble("input.ffb.rumbleGain", settings.rumbleGain);
    settings.invertDirection = config_.getBool("input.ffb.invert", settings.invertDirection);
    settings.maxForce = config_.getDouble("input.ffb.maxForce", settings.maxForce);
    return settings;
}

void ProfileStore::storeFfbSettings(const FfbSettings& settings) {
    config_.setBool("input.ffb.enabled", settings.enabled);
    config_.setDouble("input.ffb.masterGain", settings.masterGain);
    config_.setDouble("input.ffb.constantGain", settings.constantForceGain);
    config_.setDouble("input.ffb.springGain", settings.springGain);
    config_.setDouble("input.ffb.damperGain", settings.damperGain);
    config_.setDouble("input.ffb.frictionGain", settings.frictionGain);
    config_.setDouble("input.ffb.rumbleGain", settings.rumbleGain);
    config_.setBool("input.ffb.invert", settings.invertDirection);
    config_.setDouble("input.ffb.maxForce", settings.maxForce);
}

} // namespace sim::input
