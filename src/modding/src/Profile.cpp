// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include "modding/Profile.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <system_error>

#include <toml++/toml.hpp>

namespace sim::modding {
namespace {

namespace fs = std::filesystem;

std::vector<std::string> stringArray(const toml::node* node) {
    std::vector<std::string> values;
    const toml::array* array = node != nullptr ? node->as_array() : nullptr;
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

std::string readTextFile(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return {};
    }
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

} // namespace

ProfileStore::ProfileStore(std::filesystem::path file) : file_(std::move(file)) {}

bool ProfileStore::load(std::string* error) {
    profiles_.clear();
    std::error_code ec;
    if (!fs::exists(file_, ec)) {
        return true; // a missing file is an empty store
    }
    const std::string text = readTextFile(file_);
    toml::table table;
    try {
        table = toml::parse(std::string_view(text), file_.string());
    } catch (const toml::parse_error& parseError) {
        if (error != nullptr) {
            *error = file_.string() + ": " + std::string(parseError.description());
        }
        return false;
    }

    if (const toml::node* last = table.get("last")) {
        if (auto value = last->value<std::string>()) {
            lastApplied_ = *value;
        }
    }
    if (const toml::table* profiles = table.get_as<toml::table>("profiles")) {
        for (const auto& [key, value] : *profiles) {
            Profile profile;
            profile.name = std::string(key.str());
            profile.mods = stringArray(&value);
            profiles_.push_back(std::move(profile));
        }
    }
    return true;
}

bool ProfileStore::save(std::string* error) const {
    std::error_code ec;
    const fs::path parent = file_.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent, ec);
    }

    toml::table table;
    toml::table profiles;
    for (const Profile& profile : profiles_) {
        toml::array mods;
        for (const std::string& id : profile.mods) {
            mods.push_back(id);
        }
        profiles.insert(profile.name, std::move(mods));
    }
    table.insert("profiles", std::move(profiles));
    if (!lastApplied_.empty()) {
        table.insert("last", lastApplied_);
    }

    std::ofstream out(file_, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (error != nullptr) {
            *error = "cannot write '" + file_.string() + "'";
        }
        return false;
    }
    out << "# Named sets of active mods. Safe to copy to another machine.\n";
    out << table;
    if (!out) {
        if (error != nullptr) {
            *error = "cannot write '" + file_.string() + "'";
        }
        return false;
    }
    return true;
}

std::vector<std::string> ProfileStore::names() const {
    std::vector<std::string> result;
    result.reserve(profiles_.size());
    for (const Profile& profile : profiles_) {
        result.push_back(profile.name);
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::optional<Profile> ProfileStore::find(std::string_view name) const {
    for (const Profile& profile : profiles_) {
        if (profile.name == name) {
            return profile;
        }
    }
    return std::nullopt;
}

bool ProfileStore::set(Profile profile) {
    if (profile.name.empty()) {
        return false;
    }
    for (Profile& existing : profiles_) {
        if (existing.name == profile.name) {
            existing = std::move(profile);
            return true;
        }
    }
    profiles_.push_back(std::move(profile));
    return true;
}

bool ProfileStore::remove(std::string_view name) {
    const auto it = std::remove_if(profiles_.begin(), profiles_.end(),
                                   [name](const Profile& profile) { return profile.name == name; });
    if (it == profiles_.end()) {
        return false;
    }
    profiles_.erase(it, profiles_.end());
    if (lastApplied_ == name) {
        lastApplied_.clear();
    }
    return true;
}

void ProfileStore::clear() {
    profiles_.clear();
    lastApplied_.clear();
}

bool ProfileStore::exportProfile(std::string_view name, const std::filesystem::path& target, std::string* error) const {
    const std::optional<Profile> profile = find(name);
    if (!profile) {
        if (error != nullptr) {
            *error = "no profile named '" + std::string(name) + "'";
        }
        return false;
    }

    // A single-profile file is the same shape as the store, so a hand-written
    // or hand-edited file imports without a second parser.
    ProfileStore single(target);
    single.set(*profile);
    return single.save(error);
}

bool ProfileStore::importProfile(const std::filesystem::path& source, std::string* error) {
    const std::string text = readTextFile(source);
    if (text.empty()) {
        if (error != nullptr) {
            *error = "cannot read '" + source.string() + "'";
        }
        return false;
    }

    ProfileStore incoming(source);
    if (!incoming.load(error)) {
        return false;
    }
    if (incoming.all().empty()) {
        if (error != nullptr) {
            *error = "'" + source.string() + "' contains no profile";
        }
        return false;
    }
    for (const Profile& profile : incoming.all()) {
        set(profile);
    }
    return true;
}

} // namespace sim::modding
