// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "modding/Collection.hpp" // InstalledMod

namespace sim::net {
class HttpClient;
}

namespace sim::modding {

// One published version of one mod, as an index advertises it.
struct CatalogVersion {
    std::string id;
    std::string version;
    std::string downloadUrl;
    std::string sha256; // lowercase hex, checked before anything is installed
    std::uint64_t size = 0;
    std::string license;
    std::vector<std::string> types;
    std::vector<std::string> dependencies; // "id range"
    std::string changelog;
    std::string cover;
    std::string sourceUrl; // the index this entry came from
};

// A static, open index of published mods (docs/MODS.md section 6). It is only
// ever read when the user has configured an index URL: the catalog is off by
// default and no request is made on its own.
struct Catalog {
    std::string name;
    std::string description;
    std::string sourceUrl;
    std::vector<CatalogVersion> versions;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    bool ok() const { return errors.empty(); }
    // The newest published version of `id`, preferring a stable release over a
    // prerelease of the same id. Null when the catalog does not have it. With
    // `includePrerelease` false (used by the automatic update check) a prerelease
    // is never returned, so a beta on the index cannot be pushed at the user.
    const CatalogVersion* latest(std::string_view id, bool includePrerelease = true) const;
    std::vector<std::string> ids() const;
    // Entries whose id or name matches `text` (empty text matches everything),
    // optionally restricted to a content type.
    std::vector<CatalogVersion> search(std::string_view text, std::string_view type) const;
};

// Parses an index document. `source` is only used in error text.
Catalog parseCatalog(std::string_view tomlText, std::string_view source);

// Fetches and parses an index. On failure `catalog.errors` explains why (an
// unreachable index is a first-class state, not an exception).
Catalog loadCatalog(net::HttpClient& http, std::string_view url);

// A newer version available for something that is installed.
struct CatalogUpdate {
    CatalogVersion available;
    std::string installedVersion;
};

// What changed since the installed set: only genuinely newer versions are
// updates, and a prerelease is never offered over a stable release.
std::vector<CatalogUpdate> catalogUpdates(const Catalog& catalog, const std::vector<InstalledMod>& installed);

struct DownloadResult {
    bool ok = false;
    std::filesystem::path file;
    std::string error;
};

// Downloads one catalog entry to `destination` and refuses it unless its size
// and its SHA-256 match what the index promised. The result is a normal
// `.simmod` file, handed to ModInstaller like any other.
DownloadResult downloadEntry(net::HttpClient& http, const CatalogVersion& entry,
                             const std::filesystem::path& destination);

} // namespace sim::modding
