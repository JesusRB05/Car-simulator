// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/Manifest.hpp"

#include <algorithm>
#include <cctype>

#include <toml++/toml.hpp>

#include "modding/Package.hpp"

namespace sim::modding {
namespace {

constexpr int kFormatV2 = 2;

bool isLower(char c) {
    return c >= 'a' && c <= 'z';
}

bool isDigit(char c) {
    return c >= '0' && c <= '9';
}

std::string readString(const toml::table& table, std::string_view key) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<std::string>()) {
            return *value;
        }
    }
    return {};
}

std::vector<std::string> readStringArray(const toml::table& table, std::string_view key) {
    std::vector<std::string> result;
    const toml::node* node = table.get(key);
    if (node == nullptr) {
        return result;
    }
    if (const toml::array* array = node->as_array()) {
        for (const toml::node& element : *array) {
            if (auto value = element.value<std::string>()) {
                result.push_back(*value);
            }
        }
    }
    return result;
}

// Reads `dependencies`, accepting both the v2 table form
// (`{ id, version }`) and the v1 shapes (a string, a bare id, or a map of
// id -> range). `migrated` records whether a v1 shape was seen.
std::vector<Dependency> readDependencies(const toml::node* node, bool& migrated) {
    std::vector<Dependency> result;
    if (node == nullptr) {
        return result;
    }
    if (const toml::array* array = node->as_array()) {
        for (const toml::node& element : *array) {
            if (const toml::table* inner = element.as_table()) {
                Dependency dependency;
                dependency.id = readString(*inner, "id");
                const std::string range = readString(*inner, "version");
                if (!range.empty()) {
                    dependency.versionRange = range;
                }
                result.push_back(std::move(dependency));
            } else if (auto value = element.value<std::string>()) {
                // v1: "id", "id range" or "id>=1.2.0".
                migrated = true;
                std::string text = *value;
                Dependency dependency;
                const auto space = text.find(' ');
                if (space != std::string::npos) {
                    dependency.id = text.substr(0, space);
                    dependency.versionRange = text.substr(space + 1);
                } else {
                    dependency.id = text;
                }
                result.push_back(std::move(dependency));
            }
        }
    } else if (const toml::table* map = node->as_table()) {
        migrated = true;
        for (const auto& [key, value] : *map) {
            Dependency dependency;
            dependency.id = std::string(key.str());
            if (auto range = value.value<std::string>()) {
                dependency.versionRange = *range;
            }
            result.push_back(std::move(dependency));
        }
    }
    return result;
}

std::vector<Conflict> readConflicts(const toml::node* node) {
    std::vector<Conflict> result;
    if (node == nullptr) {
        return result;
    }
    if (const toml::array* array = node->as_array()) {
        for (const toml::node& element : *array) {
            if (const toml::table* inner = element.as_table()) {
                Conflict conflict;
                conflict.id = readString(*inner, "id");
                const std::string range = readString(*inner, "version");
                if (!range.empty()) {
                    conflict.versionRange = range;
                }
                conflict.reason = readString(*inner, "reason");
                result.push_back(std::move(conflict));
            } else if (auto value = element.value<std::string>()) {
                result.push_back(Conflict{*value, "*", {}});
            }
        }
    }
    return result;
}

bool isSafeRelativePath(std::string_view path) {
    if (path.empty() || path.front() == '/' || path.find('\\') != std::string_view::npos) {
        return false;
    }
    std::size_t start = 0;
    while (start <= path.size()) {
        const std::size_t end = path.find('/', start);
        const std::string_view segment =
            end == std::string_view::npos ? path.substr(start) : path.substr(start, end - start);
        if (segment.empty() || segment == ".." || segment == ".") {
            return false;
        }
        if (segment.size() >= 2 && segment[1] == ':') {
            return false; // a Windows drive letter
        }
        if (end == std::string_view::npos) {
            break;
        }
        start = end + 1;
    }
    return true;
}

} // namespace

bool isValidModId(std::string_view id) {
    // A namespace, a dot and a name: every dot-separated part starts with a
    // lowercase letter and uses lowercase letters, digits and '-'.
    if (id.find('.') == std::string_view::npos) {
        return false;
    }
    std::size_t start = 0;
    while (start <= id.size()) {
        std::size_t end = id.find('.', start);
        if (end == std::string_view::npos) {
            end = id.size();
        }
        const std::string_view part = id.substr(start, end - start);
        if (part.empty() || !isLower(part.front())) {
            return false;
        }
        if (!std::all_of(part.begin(), part.end(), [](char c) { return isLower(c) || isDigit(c) || c == '-'; })) {
            return false;
        }
        if (end == id.size()) {
            break;
        }
        start = end + 1;
    }
    return true;
}

bool isLegacyModId(std::string_view id) {
    if (id.empty() || !isLower(id.front())) {
        return false;
    }
    return std::all_of(id.begin(), id.end(), [](char c) { return isLower(c) || isDigit(c) || c == '_' || c == '-'; });
}

bool isValidContentId(std::string_view id) {
    if (id.empty() || !isLower(id.front())) {
        return false;
    }
    return std::all_of(id.begin(), id.end(), [](char c) { return isLower(c) || isDigit(c) || c == '_' || c == '-'; });
}

bool isValidContentRef(std::string_view ref) {
    const auto slash = ref.find('/');
    if (slash == std::string_view::npos) {
        return false;
    }
    const std::string_view modId = ref.substr(0, slash);
    const std::string_view contentId = ref.substr(slash + 1);
    return (isValidModId(modId) || isLegacyModId(modId)) && isValidContentId(contentId);
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

    // --- Version of the format -------------------------------------------
    const toml::node* formatNode = table.get("format");
    if (formatNode == nullptr) {
        manifest.migratedFromV1 = true;
    } else if (auto format = formatNode->value<std::int64_t>()) {
        if (*format == 1) {
            manifest.migratedFromV1 = true;
        } else if (*format != kFormatV2) {
            result.errors.push_back("[" + source + "] unsupported 'format' " + std::to_string(*format) +
                                    " (this build understands 2)");
        }
    } else {
        result.errors.push_back("[" + source + "] 'format' must be an integer");
    }

    // --- Common fields ----------------------------------------------------
    manifest.id = readString(table, "id");
    manifest.name = readString(table, "name");
    manifest.license = readString(table, "license");
    manifest.description = readString(table, "description");
    manifest.cover = readString(table, "cover");
    manifest.minGameVersion = readString(table, "min_game_version");
    manifest.entryScript = readString(table, "entry_script");

    if (const toml::node* priority = table.get("priority")) {
        manifest.priority = static_cast<int>(priority->value<std::int64_t>().value_or(0));
    }

    const std::string versionText = readString(table, "version");

    // --- Authors ----------------------------------------------------------
    manifest.authors = readStringArray(table, "authors");
    const std::string author = readString(table, "author");
    if (manifest.authors.empty() && !author.empty()) {
        manifest.authors.push_back(author);
        if (!manifest.migratedFromV1) {
            result.warnings.push_back("[" + source + "] 'author' is the v1 spelling; use 'authors = [\"...\"]'");
        }
    }

    // --- Content types ----------------------------------------------------
    manifest.types = readStringArray(table, "types");
    for (const std::string& type : manifest.types) {
        if (!isKnownContentType(type)) {
            result.errors.push_back("[" + source + "] unknown content type '" + type +
                                    "' (see docs/MOD_FORMAT.md section 3)");
        }
    }

    // --- Dependencies and conflicts --------------------------------------
    bool dependencyMigrated = false;
    manifest.dependencies = readDependencies(table.get("dependencies"), dependencyMigrated);
    manifest.conflicts = readConflicts(table.get("conflicts"));

    // --- Permissions ------------------------------------------------------
    manifest.permissions = readStringArray(table, "permissions");
    for (const std::string& permission : manifest.permissions) {
        if (!isKnownPermission(permission)) {
            result.errors.push_back("[" + source + "] unknown permission '" + permission +
                                    "' (see docs/MODS.md section 5)");
        }
    }
    const bool runsScripts = std::find(manifest.types.begin(), manifest.types.end(), "apps") != manifest.types.end() ||
                             std::find(manifest.types.begin(), manifest.types.end(), "modes") != manifest.types.end();
    if (runsScripts && manifest.permissions.empty()) {
        result.warnings.push_back("[" + source + "] ships scripts but declares no 'permissions'");
    }

    // --- Overrides and URLs ----------------------------------------------
    manifest.overrides = readStringArray(table, "overrides");
    for (const std::string& override : manifest.overrides) {
        if (!isSafeRelativePath(override)) {
            result.errors.push_back("[" + source + "] invalid override path '" + override +
                                    "' (expected a relative path inside the package)");
        }
    }
    if (const toml::table* urls = table.get_as<toml::table>("urls")) {
        manifest.website = readString(*urls, "website");
        manifest.sourceUrl = readString(*urls, "source");
    }

    // --- Validation -------------------------------------------------------
    if (manifest.id.empty()) {
        result.errors.push_back("[" + source + "] missing required field 'id'");
    } else if (isValidModId(manifest.id)) {
        // Fine.
    } else if (isLegacyModId(manifest.id)) {
        if (!manifest.migratedFromV1) {
            result.errors.push_back("[" + source + "] invalid 'id' '" + manifest.id +
                                    "' (v2 ids are namespaced, 'author.name')");
        } else {
            result.warnings.push_back("[" + source + "] mod id '" + manifest.id +
                                      "' has no namespace; new mods use 'author.name'");
        }
    } else {
        result.errors.push_back("[" + source + "] invalid 'id' '" + manifest.id +
                                "' (use lowercase letters, digits, '.' and '-')");
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

    if (manifest.authors.empty()) {
        result.errors.push_back("[" + source + "] missing required field 'authors'");
    }

    if (!manifest.migratedFromV1 && manifest.types.empty()) {
        result.errors.push_back("[" + source + "] missing required field 'types'");
    }

    for (const Dependency& dependency : manifest.dependencies) {
        if (isValidModId(dependency.id)) {
            continue;
        }
        if (dependencyMigrated && isLegacyModId(dependency.id)) {
            result.warnings.push_back("[" + source + "] dependency '" + dependency.id +
                                      "' has no namespace; new mods use 'author.name'");
            continue;
        }
        result.errors.push_back("[" + source + "] invalid dependency id '" + dependency.id + "'");
    }

    for (const Conflict& conflict : manifest.conflicts) {
        if (!isValidModId(conflict.id) && !isLegacyModId(conflict.id)) {
            result.errors.push_back("[" + source + "] invalid conflict id '" + conflict.id + "'");
        }
    }

    if (!manifest.minGameVersion.empty()) {
        if (manifest.minGameVersion != "*" && !Version::parse(manifest.minGameVersion)) {
            result.errors.push_back("[" + source + "] invalid 'min_game_version' '" + manifest.minGameVersion + "'");
        }
    }

    if (manifest.migratedFromV1) {
        result.warnings.push_back("[" + source + "] manifest v1 migrated to v2 in memory");
        if (!manifest.entryScript.empty() || manifest.priority != 0) {
            result.warnings.push_back("[" + source +
                                      "] 'entry_script'/'priority' are deprecated; "
                                      "use the 'apps/' folder and the mod load order");
        }
    } else {
        if (!manifest.entryScript.empty()) {
            result.warnings.push_back("[" + source + "] 'entry_script' is deprecated; use the 'apps/' folder");
        }
    }

    if (result.errors.empty()) {
        result.manifest = std::move(manifest);
    }
    return result;
}

} // namespace sim::modding
