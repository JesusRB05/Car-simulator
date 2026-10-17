// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sim::modding {

// A named set of active mods (and their order): "vanilla", "racing", "2026
// championship". Switching applies the whole set in one click.
struct Profile {
    std::string name;
    std::vector<std::string> mods;
};

// Named profiles, kept in one small file (`<userdata>/profiles.toml`) that the
// user can copy around. A single profile can also be exported on its own and
// imported on another machine.
class ProfileStore {
public:
    explicit ProfileStore(std::filesystem::path file);

    // A missing file is not an error: it is simply empty.
    bool load(std::string* error = nullptr);
    bool save(std::string* error = nullptr) const;

    const std::filesystem::path& file() const { return file_; }

    // Sorted by name, so the order does not depend on insertion.
    std::vector<std::string> names() const;
    std::optional<Profile> find(std::string_view name) const;
    std::vector<Profile> all() const { return profiles_; }

    bool set(Profile profile); // false when the name is empty
    bool remove(std::string_view name);
    void clear();

    // The profile applied last, remembered so the manager can show it.
    const std::string& lastApplied() const { return lastApplied_; }
    void setLastApplied(std::string_view name) { lastApplied_ = std::string(name); }

    // Writes one profile as a standalone file, and reads one back, so a setup
    // can be shared without the whole store.
    bool exportProfile(std::string_view name, const std::filesystem::path& target, std::string* error = nullptr) const;
    bool importProfile(const std::filesystem::path& source, std::string* error = nullptr);

private:
    std::filesystem::path file_;
    std::vector<Profile> profiles_;
    std::string lastApplied_;
};

} // namespace sim::modding
