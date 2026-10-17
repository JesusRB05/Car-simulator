// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "modding/Collection.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::modding {

// What a source turned out to be.
enum class InstallSource {
    Package, // a `.simmod`/`.zip` archive
    Folder,  // a folder on disk
    Unknown, // neither
};

// What installing a source would do. Produced by ModInstaller::inspect(), which
// never writes anything, so the very same plan feeds `sim --install-mod`, the
// drag-and-drop confirmation dialog and the in-game button: one behaviour, one
// thing to test.
struct InstallPlan {
    bool ok = false;
    InstallSource source = InstallSource::Unknown;
    std::filesystem::path input;   // the path the user pointed at
    bool archive = false;          // `input` is a `.simmod`/`.zip`
    std::filesystem::path payload; // folder holding mod.toml to copy
    std::filesystem::path temp;    // extraction root, empty for a folder source
    std::string modId;
    std::string name;
    std::string version;
    std::vector<std::string> types;
    // Script permissions the mod declares, shown before it is installed.
    std::vector<std::string> permissions;
    // SHA-256 of the payload, recorded by the registry on install and shown in
    // the confirmation screen.
    std::string digest;
    std::filesystem::path target; // `<mods>/<id>`
    bool replacesExisting = false;
    std::string existingVersion;
    // Set when the package is a collection: the mods it wants, to be checked
    // against what is installed with planCollection().
    std::optional<Collection> collection;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    // Human-readable, exactly what the confirmation screen shows.
    std::string summary() const;
};

struct InstallResult {
    bool ok = false;
    std::filesystem::path installed;
    std::filesystem::path backup; // where a replaced mod was moved, if any
    std::string error;
};

// The single way a mod gets onto the user's disk. Drag-and-drop, the `.simmod`
// file association, a `simmod://` link, the in-game button and `sim
// --install-mod` all funnel into install(), so there is one code path to audit
// and one place to get the safety right.
class ModInstaller {
public:
    // Hidden working folders inside the mods directory. The manager skips
    // dot-prefixed entries, so neither is ever loaded as a mod.
    static constexpr std::string_view kStagingName = ".simmod-staging";
    static constexpr std::string_view kBackupName = ".simmod-backups";
    // The installed-mod registry (ids, versions, digests and origins).
    static constexpr std::string_view kRegistryName = ".simmod-registry.toml";

    ModInstaller(vfs::VirtualFileSystem& vfs, std::filesystem::path modsDirectory);

    // Recognises a `.simmod`, a `.zip` or a folder. A source that wraps
    // everything one folder deep - as a GitHub download does - is descended
    // into (with a warning). Reads and validates the manifest; writes nothing
    // to the mods directory.
    InstallPlan inspect(const std::filesystem::path& source) const;

    // Copies the plan's payload to `<mods>/<id>` transactionally. When a mod
    // with that id is already installed it is moved to the backup area first
    // and restored if anything fails, so an interrupted install never leaves a
    // half-written mod behind.
    InstallResult install(const InstallPlan& plan);

    // Moves `<mods>/<id>` out of the way, keeping it for rollback().
    InstallResult remove(std::string_view modId);

    // Moves the last backup of `modId` back into place, discarding the current
    // install.
    InstallResult rollback(std::string_view modId);

    const std::filesystem::path& modsDirectory() const { return mods_; }

private:
    vfs::VirtualFileSystem& vfs_;
    std::filesystem::path mods_;
};

} // namespace sim::modding
