// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/Manifest.hpp"

#include <cctype>

#include <toml++/toml.hpp>

namespace sim::modding {
namespace {

std::string readString(const toml::table& table, std::string_view key) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<std::string>()) {
            return *value;
        }
    }
    return {};
}

std::vector<std::string> toStringList(const toml::node* node) {
    std::vector<std::string> result;
    if (node == nullptr) {
        return result;
    }
    if (const toml::array* array = node->as_array()) {
        for (const toml::node& element : *array) {
            if (auto value = element.value<std::string>()) {
                result.push_back(*value);
            } else if (const toml::table* inner = element.as_table()) {
                std::string entry = readString(*inner, "id");
                const std::string range = readString(*inner, "version");
                if (!range.empty()) {
                    entry += " " + range;
                }
                if (!entry.empty()) {
                    result.push_back(entry);
                }
            }
        }
    } else if (const toml::table* map = node->as_table()) {
        for (const auto& [key, value] : *map) {
            std::string entry(key.str());
            if (auto range = value.value<std::string>()) {
                entry += " " + *range;
            }
            result.push_back(entry);
        }
    }
    return result;
}

} // namespace

bool isValidModId(std::string_view id) {
    if (id.empty()) {
        return false;
    }
    const auto isLower = [](char c) { return c >= 'a' && c <= 'z'; };
    const auto isDigit = [](char c) { return c >= '0' && c <= '9'; };
    if (!isLower(id.front())) {
        return false;
    }
    for (char c : id) {
        if (!isLower(c) && !isDigit(c) && c != '_' && c != '-') {
            return false;
        }
    }
    return true;
}

ManifestResult parseManifest(std::string_view tomlText, std::string_view sourceName) {
    ManifestResult result;
    const std::string source(sourceName);

    toml::table parsed;
    try {
        parsed = toml::parse(tomlText, source);
    } catch (const toml::parse_error& error) {
        result.errors.push_back("[" + source + "] TOML parse error: " + std::string(error.description()));
        return result;
    }

    const toml::table& table = parsed;
    ModManifest manifest;

    manifest.id = readString(table, "id");
    manifest.name = readString(table, "name");
    manifest.author = readString(table, "author");
    manifest.license = readString(table, "license");
    manifest.description = readString(table, "description");
    manifest.minGameVersion = readString(table, "min_game_version");
    manifest.entryScript = readString(table, "entry_script");

    if (const toml::node* priority = table.get("priority")) {
        manifest.priority = static_cast<int>(priority->value<std::int64_t>().value_or(0));
    }

    const std::string versionText = readString(table, "version");

    // --- Validation -------------------------------------------------------
    if (manifest.id.empty()) {
        result.errors.push_back("[" + source + "] missing required field 'id'");
    } else if (!isValidModId(manifest.id)) {
        result.errors.push_back("[" + source + "] invalid 'id' '" + manifest.id +
                                "' (use lowercase letters, digits, '-' and '_')");
    }

    if (manifest.name.empty()) {
        result.errors.push_back("[" + source + "] missing required field 'name'");
    }

    if (versionText.empty()) {
        result.errors.push_back("[" + source + "] missing required field 'version'");
    } else if (auto version = Version::parse(versionText)) {
        manifest.version = *version;
    } else {
        result.errors.push_back("[" + source + "] invalid 'version' '" + versionText + "' (expected semver x.y.z)");
    }

    if (manifest.license.empty()) {
        result.errors.push_back("[" + source + "] missing required field 'license'");
    }

    for (const std::string& dependency : toStringList(table.get("dependencies"))) {
        std::string id = dependency;
        std::string range = "*";
        const auto space = dependency.find(' ');
        if (space != std::string::npos) {
            id = dependency.substr(0, space);
            range = dependency.substr(space + 1);
        }
        if (!isValidModId(id)) {
            result.errors.push_back("[" + source + "] invalid dependency id '" + id + "'");
            continue;
        }
        manifest.dependencies.push_back(Dependency{id, range});
    }

    if (!manifest.minGameVersion.empty()) {
        // A minimum game version must be parseable as a semver prefix.
        if (manifest.minGameVersion != "*" && !Version::parse(manifest.minGameVersion)) {
            result.errors.push_back("[" + source + "] invalid 'min_game_version' '" + manifest.minGameVersion + "'");
        }
    }

    if (result.errors.empty()) {
        result.manifest = std::move(manifest);
    }
    return result;
}

} // namespace sim::modding
