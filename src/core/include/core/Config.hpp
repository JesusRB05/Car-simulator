// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <toml++/toml.hpp>

namespace sim::core {

// Thin, typed wrapper over a TOML document used for user configuration.
// Keys use dot notation ("mods.active") to address nested tables. Only the
// application layer reads and writes this file; it is never part of the VFS.
class Config
{
public:
    explicit Config(std::filesystem::path file);

    // Loads the file. Returns false (and logs) if the file exists but is
    // malformed. A missing file is not an error: the config simply stays empty.
    bool load();

    // Writes the current document back to disk, creating parent directories.
    bool save() const;

    bool contains(std::string_view dottedKey) const;

    std::string getString(std::string_view key, std::string defaultValue = {}) const;
    std::int64_t getInt(std::string_view key, std::int64_t defaultValue = 0) const;
    double getDouble(std::string_view key, double defaultValue = 0.0) const;
    bool getBool(std::string_view key, bool defaultValue = false) const;
    std::vector<std::string> getStringArray(std::string_view key, std::vector<std::string> defaultValue = {}) const;

    void setString(std::string_view key, std::string_view value);
    void setInt(std::string_view key, std::int64_t value);
    void setDouble(std::string_view key, double value);
    void setBool(std::string_view key, bool value);
    void setStringArray(std::string_view key, const std::vector<std::string>& value);

    void erase(std::string_view key);

    const std::filesystem::path& file() const { return file_; }

private:
    toml::node* find(std::string_view dottedKey);
    const toml::node* find(std::string_view dottedKey) const;
    toml::table& ensureTable(std::string_view dottedPath);

    std::filesystem::path file_;
    toml::table table_;
};

} // namespace sim::core
