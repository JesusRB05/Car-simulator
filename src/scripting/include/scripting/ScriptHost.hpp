// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "scripting/LuaSandbox.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::scripting {

// Hosts mod entry scripts, one Lua state per mod. Each mod gets its own
// LuaSandbox, so a mod cannot read or overwrite another mod's globals through
// `_G`, `rawset` or a shared `sim` table; the state (and its memory ceiling and
// instruction budget) is torn down with the mod.
//
// Each loaded script can register two callbacks through the `sim` API:
//
//   sim.register_on_load(function() ... end)
//   sim.register_on_update(function(dt) ... end)
//
// It also exposes `sim.log(message)` (prefixed with the mod id) and
// `sim.read(path)` (reads through the VFS only).
class ScriptHost {
public:
    explicit ScriptHost(vfs::VirtualFileSystem& vfs);

    // Loads a Lua file from the VFS into a fresh sandbox and runs it. Returns
    // false on error; the mod stays loaded with whatever it registered.
    bool loadScript(std::string_view modId, std::string_view virtualPath);

    void callOnLoad();
    void callOnUpdate(double deltaSeconds);

    // Releases every mod's sandbox and its stored Lua functions.
    void clear();

    // The sandbox owned by `modId`, or nullptr if that mod is not loaded.
    LuaSandbox* sandbox(std::string_view modId);

    std::size_t modCount() const { return entries_.size(); }

private:
    struct Entry {
        std::string modId;
        std::string path;
        std::unique_ptr<LuaSandbox> sandbox;
        std::vector<sol::protected_function> onLoad;
        std::vector<sol::protected_function> onUpdate;
    };

    void installApi(LuaSandbox& sandbox, Entry& entry);

    vfs::VirtualFileSystem& vfs_;
    // unique_ptr keeps each Entry at a stable address, so the `sim` callbacks can
    // hold a reference to it while the vector grows.
    std::vector<std::unique_ptr<Entry>> entries_;
    std::string currentMod_ = "lua";
};

} // namespace sim::scripting
