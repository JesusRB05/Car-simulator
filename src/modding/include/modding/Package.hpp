// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <filesystem>
#include <string_view>

namespace sim::modding {

// The one place that knows the package extension and the names inside a package.
// Everything else uses these constants, so the extension can be renamed (for
// example to `.carmod`) without touching the rest of the codebase.
inline constexpr std::string_view kPackageExtension = ".simmod";
inline constexpr std::string_view kManifestName = "mod.toml";

// The mod id of the official content. It is a normal mod in every other way.
inline constexpr std::string_view kBaseModId = "base.core";

// Folder names inside a package (docs/MOD_FORMAT.md section 1).
inline constexpr std::string_view kCarsDir = "cars";
inline constexpr std::string_view kTracksDir = "tracks";
inline constexpr std::string_view kSkinsDir = "skins";
inline constexpr std::string_view kPartsDir = "parts";
inline constexpr std::string_view kTyresDir = "tyres";
inline constexpr std::string_view kSetupsDir = "setups";
inline constexpr std::string_view kSoundsDir = "sounds";
inline constexpr std::string_view kWeatherDir = "weather";
inline constexpr std::string_view kAppsDir = "apps";
inline constexpr std::string_view kModesDir = "modes";
inline constexpr std::string_view kVisualsDir = "visuals";
inline constexpr std::string_view kLocaleDir = "locale";
inline constexpr std::string_view kInputDir = "input";
inline constexpr std::string_view kPluginsDir = "plugins";
inline constexpr std::string_view kPatchesDir = "patches";

// The content types a package may declare (`types` in mod.toml).
inline constexpr std::string_view kContentTypes[] = {
    "car",  "track", "skins",   "parts",  "tyres", "sounds",  "setups",  "weather",
    "apps", "modes", "visuals", "locale", "input", "plugins", "patches", "collection",
};

// The script permissions a mod may request (`permissions` in mod.toml). The Lua
// API exposes exactly these and nothing else (docs/MODS.md section 5).
inline constexpr std::string_view kScriptPermissions[] = {
    "hud", "telemetry.read", "input.read", "vehicle.tuning", "session.control",
};

bool isKnownContentType(std::string_view type);
bool isKnownPermission(std::string_view permission);

// True when `path` names a package by extension (case-insensitive).
bool hasPackageExtension(const std::filesystem::path& path);

} // namespace sim::modding
