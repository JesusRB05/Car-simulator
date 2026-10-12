// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "modding/DependencyResolver.hpp"
#include "modding/Manifest.hpp"

namespace sim::core {
class Config;
}
namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::modding {

enum class ModState
{
    Active,
    Disabled, // valid but disabled by the user
    Invalid   // failed to load (bad manifest, missing dependency, ...)
};

struct ModStatus
{
    std::string id;
    std::string name;
    std::string version;
    std::filesystem::path source;
    bool isBase = false;
    ModState state = ModState::Disabled;
    std::vector<std::string> dependencies;
    std::vector<std::string> errors;
};

struct ModReport
{
    std::vector<ModStatus> mods;
    std::vector<std::string> messages;

    bool ok() const;
    std::string toString() const;
};

// Discovers mods, validates their manifests, resolves dependencies and mounts
// the active set onto the VFS. A single invalid mod never aborts the process:
// it is disabled with a clear error and the rest still loads.
class ModManager
{
public:
    struct Paths
    {
        std::filesystem::path baseContent;   // e.g. <root>/content/base
        std::filesystem::path modsDirectory; // e.g. <root>/mods
    };

    ModManager(vfs::VirtualFileSystem& vfs, core::Config& config, Paths paths);

    // Full pass: scan, resolve, mount, persist. Call at startup and on reload.
    void reload();

    const ModReport& report() const { return report_; }
    const std::vector<std::string>& activeIds() const { return activeIds_; }
    const Paths& paths() const { return paths_; }

    bool isActive(std::string_view id) const;
    bool setModActive(std::string_view id, bool active);

private:
    struct Discovered
    {
        ModStatus status;
        std::optional<ModManifest> manifest;
    };

    void discover();
    void mountActive();

    vfs::VirtualFileSystem& vfs_;
    core::Config& config_;
    Paths paths_;

    std::vector<Discovered> discovered_;
    std::vector<std::string> activeIds_;
    ModReport report_;
};

} // namespace sim::modding
