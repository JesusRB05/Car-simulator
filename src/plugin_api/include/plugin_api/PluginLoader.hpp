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

// Description of a native plugin after a load attempt.
struct PluginRecord
{
    std::filesystem::path path;
    std::string name;
    std::uint32_t abiVersion = 0;
    bool loaded = false;
    std::string error;

    // Runtime handles (opaque to callers).
    void* library = nullptr;
    sim_plugin_entry_fn entry = nullptr;
    sim_plugin_host host{};
    std::shared_ptr<void> context; // per-plugin api context, kept alive
};

// Loads native plugins (shared libraries implementing plugin_api.h) and drives
// their per-frame callbacks. The ABI version is verified before the plugin is
// allowed to run: the exported symbol is read first, then the struct size and
// version carried by sim_plugin_api.
class PluginLoader
{
public:
    explicit PluginLoader(vfs::VirtualFileSystem& vfs);
    ~PluginLoader();

    PluginLoader(const PluginLoader&) = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;

    // Loads a single shared library. Returns true on success.
    bool load(const std::filesystem::path& libraryPath);

    // Scans a directory for *.dll/*.so/*.dylib and loads every match.
    void loadDirectory(const std::filesystem::path& directory);

    // Calls on_shutdown on every plugin and releases every library.
    void unloadAll();

    // Drives on_update for every loaded plugin.
    void update(double deltaSeconds);

    const std::vector<PluginRecord>& plugins() const { return plugins_; }

private:
    vfs::VirtualFileSystem& vfs_;
    std::vector<PluginRecord> plugins_;
};

} // namespace sim::plugin
