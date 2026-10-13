// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "scripting/ScriptHost.hpp"

#include "core/Log.hpp"
#include "core/Version.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::scripting {

ScriptHost::ScriptHost(LuaSandbox& sandbox, vfs::VirtualFileSystem& vfs) : sandbox_(sandbox), vfs_(vfs) {
    installApi();
}

void ScriptHost::installApi() {
    sol::state_view view = sandbox_.view();

    // Everything lives in a `sim` table. (set_function("sim.log", ...) would
    // create a global literally named "sim.log", and indexing the missing
    // `sim` table afterwards raised an unprotected Lua error that killed the
    // windowed app at startup.)
    sol::table sim = view.create_named_table("sim");

    sim.set_function("log", [this](std::string message) { core::Log::forMod(currentMod_)->info("{}", message); });

    sim.set_function("read", [this](std::string path) -> std::optional<std::string> { return vfs_.readText(path); });

    sim.set_function("register_on_load", [this](sol::protected_function function) {
        if (onLoadTarget_ != nullptr) {
            onLoadTarget_->push_back(std::move(function));
        }
    });

    sim.set_function("register_on_update", [this](sol::protected_function function) {
        if (onUpdateTarget_ != nullptr) {
            onUpdateTarget_->push_back(std::move(function));
        }
    });

    sim["game_version"] = std::string(core::kVersionString);
}

bool ScriptHost::loadScript(std::string_view modId, std::string_view virtualPath) {
    const std::string path(virtualPath);
    auto text = vfs_.readText(path);
    if (!text) {
        core::Log::forMod(modId)->error("entry script '{}' not found in the VFS", path);
        return false;
    }

    Entry entry;
    entry.modId = std::string(modId);
    entry.path = path;
    entries_.push_back(std::move(entry));

    onLoadTarget_ = &entries_.back().onLoad;
    onUpdateTarget_ = &entries_.back().onUpdate;
    currentMod_ = std::string(modId);

    std::string error;
    const bool ok = sandbox_.run(*text, path, error);
    if (!ok) {
        core::Log::forMod(modId)->error("error in '{}': {}", path, error);
    }

    onLoadTarget_ = nullptr;
    onUpdateTarget_ = nullptr;
    return ok;
}

void ScriptHost::callOnLoad() {
    for (Entry& entry : entries_) {
        currentMod_ = entry.modId;
        for (auto& function : entry.onLoad) {
            sandbox_.call(function);
        }
    }
}

void ScriptHost::callOnUpdate(double deltaSeconds) {
    for (Entry& entry : entries_) {
        currentMod_ = entry.modId;
        for (auto& function : entry.onUpdate) {
            sandbox_.call(function, deltaSeconds);
        }
    }
}

void ScriptHost::clear() {
    entries_.clear();
    onLoadTarget_ = nullptr;
    onUpdateTarget_ = nullptr;
}

} // namespace sim::scripting
