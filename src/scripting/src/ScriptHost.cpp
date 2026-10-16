// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "scripting/ScriptHost.hpp"

#include "core/Log.hpp"
#include "core/Version.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::scripting {

ScriptHost::ScriptHost(vfs::VirtualFileSystem& vfs) : vfs_(vfs) {}

void ScriptHost::installApi(LuaSandbox& sandbox, Entry& entry) {
    sol::state_view view = sandbox.view();

    // Everything lives in a `sim` table. (set_function("sim.log", ...) would
    // create a global literally named "sim.log", and indexing the missing
    // `sim` table afterwards raised an unprotected Lua error that killed the
    // windowed app at startup.)
    sol::table sim = view.create_named_table("sim");

    // The callbacks capture their own Entry, so a mod can only register into its
    // own sandbox: there is no shared target pointer that a later mod could
    // hijack.
    sim.set_function("log", [&entry](std::string message) { core::Log::forMod(entry.modId)->info("{}", message); });

    sim.set_function("read", [this](std::string path) -> std::optional<std::string> { return vfs_.readText(path); });

    sim.set_function("register_on_load",
                     [&entry](sol::protected_function function) { entry.onLoad.push_back(std::move(function)); });

    sim.set_function("register_on_update",
                     [&entry](sol::protected_function function) { entry.onUpdate.push_back(std::move(function)); });

    sim["game_version"] = std::string(core::kVersionString);
}

bool ScriptHost::loadScript(std::string_view modId, std::string_view virtualPath) {
    const std::string path(virtualPath);
    auto text = vfs_.readText(path);
    if (!text) {
        core::Log::forMod(modId)->error("entry script '{}' not found in the VFS", path);
        return false;
    }

    // Each mod gets its own Lua state; the vector of unique_ptr keeps the Entry
    // address stable while more mods are loaded.
    auto entry = std::make_unique<Entry>();
    entry->modId = std::string(modId);
    entry->path = path;
    entry->sandbox = std::make_unique<LuaSandbox>();
    installApi(*entry->sandbox, *entry);

    Entry& stored = *entry;
    entries_.push_back(std::move(entry));

    currentMod_ = stored.modId;

    std::string error;
    const bool ok = stored.sandbox->run(*text, path, error);
    if (!ok) {
        core::Log::forMod(modId)->error("error in '{}': {}", path, error);
    }
    return ok;
}

void ScriptHost::callOnLoad() {
    for (auto& entry : entries_) {
        currentMod_ = entry->modId;
        for (auto& function : entry->onLoad) {
            entry->sandbox->call(function);
        }
    }
}

void ScriptHost::callOnUpdate(double deltaSeconds) {
    for (auto& entry : entries_) {
        currentMod_ = entry->modId;
        for (auto& function : entry->onUpdate) {
            entry->sandbox->call(function, deltaSeconds);
        }
    }
}

void ScriptHost::clear() {
    entries_.clear();
}

LuaSandbox* ScriptHost::sandbox(std::string_view modId) {
    for (auto& entry : entries_) {
        if (entry->modId == modId) {
            return entry->sandbox.get();
        }
    }
    return nullptr;
}

} // namespace sim::scripting
