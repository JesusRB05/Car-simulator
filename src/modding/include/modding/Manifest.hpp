// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "modding/Version.hpp"

namespace sim::modding {

struct Dependency
{
    std::string id;
    std::string versionRange = "*";
};

// Parsed contents of a mod's `mod.toml`. See docs/MODDING.md for the schema.
struct ModManifest
{
    std::string id;
    std::string name;
    Version version;
    std::string author;
    std::string license;
    std::string description;
    std::string minGameVersion = "*";
    std::vector<Dependency> dependencies;
    int priority = 0;
    std::string entryScript;
};

struct ManifestResult
{
    std::optional<ModManifest> manifest;
    std::vector<std::string> errors;

    bool ok() const { return manifest.has_value() && errors.empty(); }
};

// A valid mod id: lowercase letters, digits, '_' and '-', starting with a letter.
bool isValidModId(std::string_view id);

// Parses and validates a mod manifest. `sourceName` is only used in error text.
ManifestResult parseManifest(std::string_view tomlText, std::string_view sourceName);

} // namespace sim::modding
