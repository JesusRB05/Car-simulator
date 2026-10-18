// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "modding/DependencyResolver.hpp"
#include "modding/Fingerprint.hpp"
#include "modding/Manifest.hpp"
#include "modding/Profile.hpp"

namespace sim::core {
class Config;
}
namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::modding {

enum class ModState {
    Active,
    Disabled, // valid but disabled by the user
    Invalid   // failed to load (bad manifest, missing dependency, ...)
};

struct ModStatus {
    std::string id;
    std::string name;
    std::string version;
    std::filesystem::path source;
    bool isBase = false;
    ModState state = ModState::Disabled;
    std::vector<std::string> types;
    std::vector<std::string> dependencies;
    // Whole files this mod declares it replaces (`overrides` in mod.toml), used
    // by the conflict detector.
    std::vector<std::string> overrides;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    bool migratedFromV1 = false;
};

struct ModReport {
    std::vector<ModStatus> mods;
    std::vector<std::string> messages;

    bool ok() const;
    std::string toString() const;
};

// Discovers mods, validates their manifests, resolves dependencies and mounts
// the active set onto the VFS. A single invalid mod never aborts the process:
// it is disabled with a clear error and the rest still loads.
class ModManager {
public:
    struct Paths {
        std::filesystem::path baseContent;   // e.g. <root>/content/base
        std::filesystem::path modsDirectory; // where new mods are installed and scanned
        // Also scanned, below `modsDirectory`. The bundled `mods/` folder lives
        // here so the official example mod still loads next to the user's own
        // mods, which are kept in the per-user data directory.
        std::vector<std::filesystem::path> extraModDirectories;
        // Named active sets, e.g. `<userdata>/profiles.toml`. Empty disables
        // profiles for this manager.
        std::filesystem::path profilesFile;
    };

    ModManager(vfs::VirtualFileSystem& vfs, core::Config& config, Paths paths);

    // Full pass: scan, resolve, mount, persist. Call at startup and on reload.
    void reload();

    const ModReport& report() const { return report_; }
    const std::vector<std::string>& activeIds() const { return activeIds_; }
    const Paths& paths() const { return paths_; }

    bool isActive(std::string_view id) const;
    bool setModActive(std::string_view id, bool active);

    // Named sets of active mods ("profiles"). Switching a profile replaces the
    // active set and reloads; the current set can be saved under a name, and a
    // profile can be exported to a file and imported back.
    const ProfileStore& profiles() const { return profiles_; }
    bool applyProfile(std::string_view name);
    bool saveProfile(std::string_view name);
    bool deleteProfile(std::string_view name);

    // The content a peer must have to simulate with us (docs/MODS.md section 8):
    // every active mod that changes the simulation, with a digest of its files.
    // Skins, sounds and HUD apps are left out on purpose.
    std::vector<ContentSource> contentSources() const;
    ContentFingerprint contentFingerprint() const;

    // Developer mode (docs/MODS.md section 9): watch the sources of unpacked
    // mods and report when one changed, so the app can reload without a restart.
    void setDeveloper(bool enabled);
    bool developer() const { return developer_; }
    // True once after a watched source changed since the previous call (and the
    // first call after enabling, when the baseline is taken).
    bool pollDevelopment();

    // The id of the mod that provides a virtual file, used to build namespaced
    // content references (`<mod-id>/<content-id>`). Empty when the file comes
    // from the base content or cannot be resolved.
    std::string modIdFor(std::string_view virtualPath) const;

private:
    struct Discovered {
        ModStatus status;
        std::optional<ModManifest> manifest;
    };

    void discover();
    void mountActive();
    // File stamps of every watched source, used to detect a change.
    void refreshWatch();

    vfs::VirtualFileSystem& vfs_;
    core::Config& config_;
    Paths paths_;

    std::vector<Discovered> discovered_;
    std::vector<std::string> activeIds_;
    ModReport report_;
    ProfileStore profiles_;
    bool developer_ = false;
    std::vector<std::pair<std::filesystem::path, std::uint64_t>> watch_;
};

} // namespace sim::modding
