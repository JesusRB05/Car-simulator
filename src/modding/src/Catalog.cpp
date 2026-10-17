// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include "modding/Catalog.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <memory>
#include <set>
#include <system_error>

#include <toml++/toml.hpp>

#include "core/Sha256.hpp"
#include "modding/Manifest.hpp"
#include "net/HttpClient.hpp"

namespace sim::modding {
namespace {

namespace fs = std::filesystem;

std::string stringField(const toml::table& table, std::string_view key) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<std::string>()) {
            return *value;
        }
    }
    return {};
}

std::uint64_t intField(const toml::table& table, std::string_view key) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<std::int64_t>()) {
            return static_cast<std::uint64_t>(*value);
        }
    }
    return 0;
}

std::vector<std::string> stringArrayField(const toml::table& table, std::string_view key) {
    std::vector<std::string> values;
    const toml::array* array = table.get_as<toml::array>(key);
    if (array == nullptr) {
        return values;
    }
    for (const toml::node& element : *array) {
        if (auto value = element.value<std::string>()) {
            values.push_back(*value);
        }
    }
    return values;
}

bool isHexDigest(std::string_view text) {
    if (text.size() != 64) {
        return false;
    }
    return std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isxdigit(c) != 0; });
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

} // namespace

Catalog parseCatalog(std::string_view tomlText, std::string_view source) {
    Catalog catalog;
    catalog.sourceUrl = std::string(source);
    const std::string where(source);

    toml::table table;
    try {
        table = toml::parse(tomlText, source);
    } catch (const toml::parse_error& error) {
        catalog.errors.push_back(where + ": " + std::string(error.description()));
        return catalog;
    }

    catalog.name = stringField(table, "name");
    catalog.description = stringField(table, "description");

    std::set<std::string> seen;
    const toml::array* versions = table.get_as<toml::array>("version");
    if (versions == nullptr) {
        catalog.errors.push_back(where + ": the index has no [[version]] entries");
        return catalog;
    }

    int index = 0;
    for (const toml::node& node : *versions) {
        ++index;
        const toml::table* entry = node.as_table();
        if (entry == nullptr) {
            catalog.errors.push_back(where + ": [[version]] #" + std::to_string(index) + " is not a table");
            continue;
        }

        CatalogVersion parsed;
        parsed.id = stringField(*entry, "id");
        parsed.version = stringField(*entry, "version");
        parsed.downloadUrl = stringField(*entry, "download");
        parsed.sha256 = lower(stringField(*entry, "sha256"));
        parsed.size = intField(*entry, "size");
        parsed.license = stringField(*entry, "license");
        parsed.changelog = stringField(*entry, "changelog");
        parsed.cover = stringField(*entry, "cover");
        parsed.types = stringArrayField(*entry, "types");
        parsed.dependencies = stringArrayField(*entry, "dependencies");
        parsed.sourceUrl = catalog.sourceUrl;

        const std::string prefix = where + ": [[version]] #" + std::to_string(index) + " ";
        if (parsed.id.empty() || (!isValidModId(parsed.id) && !isLegacyModId(parsed.id))) {
            catalog.errors.push_back(prefix + "has a missing or invalid id '" + parsed.id + "'");
            continue;
        }
        if (!Version::parse(parsed.version)) {
            catalog.errors.push_back(prefix + "('" + parsed.id + "') has an invalid version '" + parsed.version + "'");
            continue;
        }
        if (parsed.downloadUrl.empty()) {
            catalog.errors.push_back(prefix + "('" + parsed.id + "') has no download URL");
            continue;
        }
        if (!isHexDigest(parsed.sha256)) {
            catalog.errors.push_back(prefix + "('" + parsed.id + "') needs a SHA-256 digest of 64 hex characters");
            continue;
        }
        if (parsed.size == 0) {
            catalog.warnings.push_back(prefix + "('" + parsed.id + "') declares no size; only the digest is checked");
        }
        const std::string key = parsed.id + "@" + parsed.version;
        if (!seen.insert(key).second) {
            catalog.errors.push_back(prefix + "duplicates '" + key + "'");
            continue;
        }
        catalog.versions.push_back(std::move(parsed));
    }
    return catalog;
}

Catalog loadCatalog(net::HttpClient& http, std::string_view url) {
    const net::HttpResponse response = http.get(url);
    if (!response.ok()) {
        Catalog catalog;
        catalog.sourceUrl = std::string(url);
        if (response.error.empty()) {
            catalog.errors.push_back("cannot read '" + std::string(url) + "': HTTP " + std::to_string(response.status));
        } else {
            catalog.errors.push_back("cannot read '" + std::string(url) + "': " + response.error);
        }
        return catalog;
    }
    return parseCatalog(response.body, url);
}

const CatalogVersion* Catalog::latest(std::string_view id, bool includePrerelease) const {
    const CatalogVersion* best = nullptr;
    Version bestVersion;
    for (const CatalogVersion& entry : versions) {
        if (entry.id != id) {
            continue;
        }
        const std::optional<Version> version = Version::parse(entry.version);
        if (!version) {
            continue;
        }
        if (!includePrerelease && !version->prerelease().empty()) {
            continue;
        }
        if (best == nullptr) {
            best = &entry;
            bestVersion = *version;
            continue;
        }
        const bool higher = *version > bestVersion;
        // On an equal version a stable release wins over a prerelease.
        const bool stableOverPrerelease =
            *version == bestVersion && !bestVersion.prerelease().empty() && version->prerelease().empty();
        if (higher || stableOverPrerelease) {
            best = &entry;
            bestVersion = *version;
        }
    }
    return best;
}

std::vector<std::string> Catalog::ids() const {
    std::set<std::string> unique;
    for (const CatalogVersion& entry : versions) {
        unique.insert(entry.id);
    }
    return {unique.begin(), unique.end()};
}

std::vector<CatalogVersion> Catalog::search(std::string_view text, std::string_view type) const {
    std::vector<CatalogVersion> found;
    const std::string needle = lower(std::string(text));
    for (const CatalogVersion& entry : versions) {
        if (!type.empty() && std::find(entry.types.begin(), entry.types.end(), type) == entry.types.end()) {
            continue;
        }
        if (!needle.empty() && lower(entry.id).find(needle) == std::string::npos) {
            continue;
        }
        found.push_back(entry);
    }
    return found;
}

std::vector<CatalogUpdate> catalogUpdates(const Catalog& catalog, const std::vector<InstalledMod>& installed) {
    std::vector<CatalogUpdate> updates;
    for (const InstalledMod& mod : installed) {
        // Only stable releases are offered automatically: the user asked for
        // "is there something newer", not for the bleeding edge.
        const CatalogVersion* available = catalog.latest(mod.id, false);
        if (available == nullptr) {
            continue;
        }
        const std::optional<Version> candidate = Version::parse(available->version);
        if (!candidate) {
            continue;
        }
        const std::optional<Version> current = Version::parse(mod.version);
        if (current && !(*candidate > *current)) {
            continue;
        }
        updates.push_back({*available, mod.version});
    }
    return updates;
}

DownloadResult downloadEntry(net::HttpClient& http, const CatalogVersion& entry,
                             const std::filesystem::path& destination) {
    DownloadResult result;

    const net::HttpResponse response = http.get(entry.downloadUrl);
    if (!response.ok()) {
        if (response.error.empty()) {
            result.error = "cannot download '" + entry.downloadUrl + "': HTTP " + std::to_string(response.status);
        } else {
            result.error = "cannot download '" + entry.downloadUrl + "': " + response.error;
        }
        return result;
    }

    if (entry.size != 0 && response.body.size() != entry.size) {
        result.error = "'" + entry.id + "' downloaded " + std::to_string(response.body.size()) +
                       " bytes but the index " + "promises " + std::to_string(entry.size) + "; the file was discarded";
        return result;
    }

    // The digest is the whole point: verify before anything reaches the disk.
    const std::string digest = core::sha256Hex(response.body.data(), response.body.size());
    if (digest != entry.sha256) {
        result.error = "'" + entry.id + "' does not match the index: expected SHA-256 " + entry.sha256 + ", got " +
                       digest + "; the file was discarded";
        return result;
    }

    std::error_code ec;
    const fs::path parent = destination.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent, ec);
    }
    std::ofstream out(destination, std::ios::binary | std::ios::trunc);
    if (!out) {
        result.error = "cannot write '" + destination.string() + "'";
        return result;
    }
    out.write(response.body.data(), static_cast<std::streamsize>(response.body.size()));
    if (!out) {
        result.error = "cannot write '" + destination.string() + "'";
        return result;
    }
    result.ok = true;
    result.file = destination;
    return result;
}

} // namespace sim::modding
