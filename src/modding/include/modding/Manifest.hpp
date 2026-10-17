// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "modding/Version.hpp"

namespace sim::modding {

struct Dependency {
    std::string id;
    std::string versionRange = "*";
};

// A mod the manifest declares it cannot live with (`conflicts`). The reason is
// shown in the manager.
struct Conflict {
    std::string id;
    std::string versionRange = "*";
    std::string reason;
};

// Parsed contents of a mod's `mod.toml`, format 2. See docs/MOD_FORMAT.md.
// A manifest without `format` is migrated from v1 in memory: `author` becomes
// `authors`, string dependencies become tables, `entry_script` is kept as a
// legacy entry point and the id is kept verbatim (even without a namespace).
struct ModManifest {
    int format = 2;

    std::string id;
    std::string name;
    Version version;
    std::vector<std::string> authors;
    std::string license;
    std::string description;
    std::string cover;
    std::string minGameVersion = "*";

    std::vector<std::string> types;
    std::vector<Dependency> dependencies;
    std::vector<Conflict> conflicts;

    // Script permissions (`hud`, `telemetry.read`, ...). Only these are exposed
    // to the mod's Lua sandbox.
    std::vector<std::string> permissions;

    // Paths inside this package that replace a whole file of another mod. A
    // full-file override is only allowed when declared here.
    std::vector<std::string> overrides;

    std::string website;
    std::string sourceUrl;

    // True when this manifest came from a v1 file and was upgraded in memory.
    bool migratedFromV1 = false;

    // Legacy v1 fields, reported as deprecated.
    std::string entryScript;
    int priority = 0;
};

struct ManifestResult {
    std::optional<ModManifest> manifest;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    bool ok() const { return manifest.has_value() && errors.empty(); }
};

// A valid v2 mod id: a namespace and a name, `<author>.<name>`, each part
// starting with a lowercase letter and using lowercase letters, digits and '-'.
bool isValidModId(std::string_view id);

// The pre-v2 shape (no namespace): lowercase letters, digits, '_' and '-',
// starting with a letter. Accepted for migrated mods, with a warning.
bool isLegacyModId(std::string_view id);

// A content id (a folder name inside a package).
bool isValidContentId(std::string_view id);

// A content reference, `mod-id/content-id` (for example `base.core/voyager`).
bool isValidContentRef(std::string_view ref);

// Parses and validates a mod manifest. `sourceName` is only used in error text.
ManifestResult parseManifest(std::string_view tomlText, std::string_view sourceName);

} // namespace sim::modding
