// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "scripting/LuaSandbox.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::scripting {

// Hosts mod entry scripts inside a shared sandbox. Each loaded script can
// register two callbacks through the `sim` API:
//
//   sim.register_on_load(function() ... end)
//   sim.register_on_update(function(dt) ... end)
//
// It also exposes `sim.log(message)` (prefixed with the mod id) and
// `sim.read(path)` (reads through the VFS only).
class ScriptHost
{
public:
    ScriptHost(LuaSandbox& sandbox, vfs::VirtualFileSystem& vfs);

    // Loads a Lua file from the VFS and runs it. Returns false on error.
    bool loadScript(std::string_view modId, std::string_view virtualPath);

    void callOnLoad();
    void callOnUpdate(double deltaSeconds);

    // Releases every stored Lua function. Must be called before the sandbox dies.
    void clear();

    LuaSandbox& sandbox() { return sandbox_; }

private:
    struct Entry
    {
        std::string modId;
        std::string path;
        std::vector<sol::protected_function> onLoad;
        std::vector<sol::protected_function> onUpdate;
    };

    void installApi();

    LuaSandbox& sandbox_;
    vfs::VirtualFileSystem& vfs_;
    std::vector<Entry> entries_;
    std::string currentMod_ = "lua";
    std::vector<sol::protected_function>* onLoadTarget_ = nullptr;
    std::vector<sol::protected_function>* onUpdateTarget_ = nullptr;
};

} // namespace sim::scripting
