// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include "modding/Registry.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

#include <toml++/toml.hpp>

#include "core/Sha256.hpp"

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

} // namespace

std::string payloadDigest(const std::filesystem::path& directory) {
    std::error_code ec;
    std::vector<std::string> relativePaths;
    for (const fs::recursive_directory_iterator::value_type& entry : fs::recursive_directory_iterator(directory, ec)) {
        if (!entry.is_regular_file(ec)) {
            continue;
        }
        relativePaths.push_back(entry.path().lexically_relative(directory).generic_string());
    }
    // The order the filesystem happened to return is not part of the content.
    std::sort(relativePaths.begin(), relativePaths.end());

    core::Sha256 hash;
    for (const std::string& relative : relativePaths) {
        const fs::path file = directory / fs::path(relative);
        std::ifstream in(file, std::ios::binary);
        if (!in) {
            continue;
        }
        std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const std::string size = std::to_string(bytes.size());
        hash.update(relative.data(), relative.size());
        hash.update("\n", 1);
        hash.update(size.data(), size.size());
        hash.update("\n", 1);
        hash.update(bytes.data(), bytes.size());
        hash.update("\n", 1);
    }
    return hash.hexDigest();
}

ModRegistry::ModRegistry(std::filesystem::path file) : file_(std::move(file)) {}

bool ModRegistry::load(std::string* error) {
    entries_.clear();
    std::error_code ec;
    if (!fs::exists(file_, ec)) {
        return true; // a missing registry is an empty registry
    }
    std::ifstream in(file_, std::ios::binary);
    if (!in) {
        if (error != nullptr) {
            *error = "cannot read '" + file_.string() + "'";
        }
        return false;
    }
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    toml::table table;
    try {
        table = toml::parse(std::string_view(text), file_.string());
    } catch (const toml::parse_error& parseError) {
        if (error != nullptr) {
            *error = file_.string() + ": " + std::string(parseError.description());
        }
        return false;
    }

    if (const toml::array* mods = table.get_as<toml::array>("mod")) {
        for (const toml::node& node : *mods) {
            const toml::table* entry = node.as_table();
            if (entry == nullptr) {
                continue;
            }
            RegistryEntry record;
            record.id = stringField(*entry, "id");
            record.version = stringField(*entry, "version");
            record.digest = stringField(*entry, "digest");
            record.source = stringField(*entry, "source");
            if (!record.id.empty()) {
                entries_.push_back(std::move(record));
            }
        }
    }
    return true;
}

bool ModRegistry::save(std::string* error) const {
    std::error_code ec;
    const fs::path parent = file_.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent, ec);
    }

    toml::table table;
    toml::array mods;
    for (const RegistryEntry& entry : entries_) {
        toml::table record;
        record.insert("id", entry.id);
        record.insert("version", entry.version);
        record.insert("digest", entry.digest);
        if (!entry.source.empty()) {
            record.insert("source", entry.source);
        }
        mods.push_back(std::move(record));
    }
    table.insert("mod", std::move(mods));

    std::ofstream out(file_, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (error != nullptr) {
            *error = "cannot write '" + file_.string() + "'";
        }
        return false;
    }
    out << "# Installed mod registry, written by ModInstaller. Do not edit.\n";
    out << table;
    if (!out) {
        if (error != nullptr) {
            *error = "cannot write '" + file_.string() + "'";
        }
        return false;
    }
    return true;
}

std::optional<RegistryEntry> ModRegistry::find(std::string_view id) const {
    for (const RegistryEntry& entry : entries_) {
        if (entry.id == id) {
            return entry;
        }
    }
    return std::nullopt;
}

void ModRegistry::upsert(RegistryEntry entry) {
    for (RegistryEntry& existing : entries_) {
        if (existing.id == entry.id) {
            existing = std::move(entry);
            return;
        }
    }
    entries_.push_back(std::move(entry));
}

void ModRegistry::remove(std::string_view id) {
    entries_.erase(
        std::remove_if(entries_.begin(), entries_.end(), [id](const RegistryEntry& entry) { return entry.id == id; }),
        entries_.end());
}

bool ModRegistry::verify(std::string_view id, const std::filesystem::path& directory) const {
    const std::optional<RegistryEntry> entry = find(id);
    if (!entry || entry->digest.empty()) {
        return false;
    }
    return payloadDigest(directory) == entry->digest;
}

} // namespace sim::modding
