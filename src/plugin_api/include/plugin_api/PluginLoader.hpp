// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "plugin_api/plugin_api.h"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::plugin {

enum class PluginState {
    Loaded,
    // Present but deliberately not loaded: the master switch is off or the file
    // is not on the allowlist.
    Skipped,
    // Approved and attempted, but the library could not be used.
    Failed
};

// Description of a native plugin after a load attempt.
struct PluginRecord {
    std::filesystem::path path;
    std::string name;
    std::string sha256; // lowercase hex digest of the file on disk
    std::uint32_t abiVersion = 0;
    PluginState state = PluginState::Skipped;
    std::string error;

    // Runtime handles (opaque to callers).
    void* library = nullptr;
    sim_plugin_entry_fn entry = nullptr;
    sim_plugin_host host{};
    std::shared_ptr<void> context; // per-plugin api context, kept alive
};

// Which native plugins may be loaded. Loading is **opt-in**: `enabled` must be
// true and the file name must appear in `allow`. Both come from config.toml
// (`plugins.enabled`, `plugins.allow`) and default to off / empty, so simply
// dropping a library into `plugins/` never runs it.
struct PluginLoadOptions {
    bool enabled = false;
    std::vector<std::string> allow;
};

// Loads native plugins (shared libraries implementing plugin_api.h) and drives
// their per-frame callbacks. The ABI version is verified before the plugin is
// allowed to run: the exported symbol is read first, then the struct size and
// version carried by sim_plugin_api.
class PluginLoader {
public:
    explicit PluginLoader(vfs::VirtualFileSystem& vfs);
    ~PluginLoader();

    PluginLoader(const PluginLoader&) = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;

    // Loads a single shared library, bypassing the allowlist. Returns true on
    // success. `loadDirectory` is the gate that decides which files reach here.
    bool load(const std::filesystem::path& libraryPath);

    // Scans a directory for *.dll/*.so/*.dylib. Every library is fingerprinted
    // (SHA-256) and recorded; only those the options allow are actually loaded.
    // With `options.enabled == false` nothing is loaded and every library is
    // recorded as Skipped.
    void loadDirectory(const std::filesystem::path& directory, const PluginLoadOptions& options);

    // Calls on_shutdown on every plugin and releases every library.
    void unloadAll();

    // Drives on_update for every loaded plugin.
    void update(double deltaSeconds);

    const std::vector<PluginRecord>& plugins() const { return plugins_; }

    // Whether the last loadDirectory had native plugins enabled.
    bool enabled() const { return enabled_; }

    // Whether any discovered library was skipped because it is not trusted.
    bool hasSkipped() const;

private:
    vfs::VirtualFileSystem& vfs_;
    bool enabled_ = false;
    std::vector<PluginRecord> plugins_;
};

} // namespace sim::plugin
