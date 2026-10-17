// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "modding/Version.hpp"

namespace sim::modding {

// One mod a collection asks for. The range uses the same syntax as a
// dependency (`satisfies`), and `optional` entries are offered but never
// blocking.
struct CollectionEntry {
    std::string modId;
    std::string versionRange = "*";
    bool optional = false;
    std::string reason; // why it is in the pack, shown in the manager
};

// A collection (modpack, championship): a mod that only lists other mods and
// can carry a session configuration. Its `collection.toml` sits next to
// `mod.toml`; `id`/`name` may be omitted there and taken from the manifest.
struct Collection {
    std::string id;
    std::string name;
    std::string description;
    std::string session; // optional session configuration file inside the pack
    std::vector<CollectionEntry> entries;
};

struct CollectionResult {
    std::optional<Collection> collection;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    bool ok() const { return collection.has_value() && errors.empty(); }
};

// Parses a `collection.toml`. `source` is only used in error text.
CollectionResult parseCollection(std::string_view tomlText, std::string_view source);

// A mod as it is installed today, for checking a collection against reality.
struct InstalledMod {
    std::string id;
    std::string version;
};

// One entry checked against the installed set.
struct CollectionRequirement {
    enum class State {
        Satisfied,
        Missing,
        VersionMismatch,
    };

    CollectionEntry entry;
    State state = State::Missing;
    std::string installedVersion; // empty when the mod is absent
    std::string message;          // human-readable, for the manager

    bool satisfied() const { return state == State::Satisfied; }
    // An unmet optional entry is reported but never blocks the pack.
    bool blocking() const { return !satisfied() && !entry.optional; }
};

struct CollectionPlan {
    std::vector<CollectionRequirement> requirements;
    // Every blocking requirement is satisfied: the pack can be played.
    bool complete() const;
    // The ids that still have to be installed (for the update/download flow).
    std::vector<std::string> missingIds() const;
    // One human-readable line per unmet requirement.
    std::vector<std::string> messages() const;
};

// Checks a collection against the installed mods.
CollectionPlan planCollection(const Collection& collection, const std::vector<InstalledMod>& installed);

} // namespace sim::modding
