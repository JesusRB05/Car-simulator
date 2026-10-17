// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/Collection.hpp"

#include <string>

#include <toml++/toml.hpp>

#include "modding/Manifest.hpp"

namespace sim::modding {
namespace {

std::string stringField(const toml::table& table, std::string_view key) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<std::string>()) {
            return *value;
        }
    }
    return {};
}

bool boolField(const toml::table& table, std::string_view key, bool fallback) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<bool>()) {
            return *value;
        }
    }
    return fallback;
}

} // namespace

CollectionResult parseCollection(std::string_view tomlText, std::string_view source) {
    CollectionResult result;
    const std::string where(source);

    toml::table table;
    try {
        table = toml::parse(tomlText, source);
    } catch (const toml::parse_error& error) {
        result.errors.push_back(where + ": " + std::string(error.description()));
        return result;
    }

    Collection collection;
    collection.id = stringField(table, "id");
    collection.name = stringField(table, "name");
    collection.description = stringField(table, "description");
    collection.session = stringField(table, "session");

    if (const toml::array* mods = table.get_as<toml::array>("mod")) {
        int index = 0;
        for (const toml::node& node : *mods) {
            ++index;
            const toml::table* entry = node.as_table();
            if (entry == nullptr) {
                result.errors.push_back(where + ": [[mod]] #" + std::to_string(index) + " is not a table");
                continue;
            }
            CollectionEntry parsed;
            parsed.modId = stringField(*entry, "id");
            parsed.versionRange = stringField(*entry, "version");
            if (parsed.versionRange.empty()) {
                parsed.versionRange = "*";
            }
            parsed.optional = boolField(*entry, "optional", false);
            parsed.reason = stringField(*entry, "reason");

            if (parsed.modId.empty()) {
                result.errors.push_back(where + ": [[mod]] #" + std::to_string(index) + " has no 'id'");
                continue;
            }
            if (!isValidModId(parsed.modId) && !isLegacyModId(parsed.modId)) {
                result.errors.push_back(where + ": [[mod]] #" + std::to_string(index) + " has an invalid id '" +
                                        parsed.modId + "'");
                continue;
            }
            collection.entries.push_back(std::move(parsed));
        }
    }

    if (result.errors.empty() && collection.entries.empty()) {
        result.warnings.push_back(where + ": the collection lists no mods");
    }
    if (result.errors.empty()) {
        result.collection = std::move(collection);
    }
    return result;
}

bool CollectionPlan::complete() const {
    for (const CollectionRequirement& requirement : requirements) {
        if (requirement.blocking()) {
            return false;
        }
    }
    return true;
}

std::vector<std::string> CollectionPlan::missingIds() const {
    std::vector<std::string> ids;
    for (const CollectionRequirement& requirement : requirements) {
        if (requirement.state == CollectionRequirement::State::Missing) {
            ids.push_back(requirement.entry.modId);
        }
    }
    return ids;
}

std::vector<std::string> CollectionPlan::messages() const {
    std::vector<std::string> lines;
    for (const CollectionRequirement& requirement : requirements) {
        if (requirement.satisfied()) {
            continue;
        }
        lines.push_back(requirement.message);
    }
    return lines;
}

CollectionPlan planCollection(const Collection& collection, const std::vector<InstalledMod>& installed) {
    CollectionPlan plan;
    for (const CollectionEntry& entry : collection.entries) {
        CollectionRequirement requirement;
        requirement.entry = entry;

        const InstalledMod* found = nullptr;
        for (const InstalledMod& candidate : installed) {
            if (candidate.id == entry.modId) {
                found = &candidate;
                break;
            }
        }

        if (found == nullptr) {
            requirement.state = CollectionRequirement::State::Missing;
            requirement.message =
                "the collection needs '" + entry.modId + "' (" + entry.versionRange + "), which is not installed";
        } else {
            requirement.installedVersion = found->version;
            const std::optional<Version> version = Version::parse(found->version);
            if (version && satisfies(entry.versionRange, *version)) {
                requirement.state = CollectionRequirement::State::Satisfied;
            } else {
                requirement.state = CollectionRequirement::State::VersionMismatch;
                requirement.message = "the collection needs '" + entry.modId + "' " + entry.versionRange + " but " +
                                      found->version + " is installed";
            }
        }
        if (entry.optional && !requirement.satisfied()) {
            requirement.message += " (optional)";
        }
        if (!entry.reason.empty()) {
            requirement.message += ": " + entry.reason;
        }
        plan.requirements.push_back(std::move(requirement));
    }
    return plan;
}

} // namespace sim::modding
