// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sim::modding {

// Lowercase-hex SHA-256 over the whole payload of a directory: every file, in
// order of its '/'-separated relative path, hashing "path\nsize\nbytes\n". It
// ignores timestamps and directory order, so the same content always hashes to
// the same digest on every platform.
std::string payloadDigest(const std::filesystem::path& directory);

// What the installer remembers about an installed mod.
struct RegistryEntry {
    std::string id;
    std::string version;
    std::string digest; // payloadDigest() of the installed files
    std::string source; // where it came from (a path or a URL), for updates
};

// The installed-mod registry, kept in `<mods>/.simmod-registry.toml`. It is the
// only record of where a mod came from and of what it looked like when it was
// installed, so the manager can show a mod's origin and detect a payload that
// changed behind the installer's back.
class ModRegistry {
public:
    explicit ModRegistry(std::filesystem::path file);

    // Reads the file. A missing file is not an error: it is simply empty.
    bool load(std::string* error = nullptr);
    bool save(std::string* error = nullptr) const;

    std::optional<RegistryEntry> find(std::string_view id) const;
    void upsert(RegistryEntry entry);
    void remove(std::string_view id);
    const std::vector<RegistryEntry>& entries() const { return entries_; }

    const std::filesystem::path& file() const { return file_; }

    // True when `directory` still hashes to the digest recorded for `id`.
    bool verify(std::string_view id, const std::filesystem::path& directory) const;

private:
    std::filesystem::path file_;
    std::vector<RegistryEntry> entries_;
};

} // namespace sim::modding
