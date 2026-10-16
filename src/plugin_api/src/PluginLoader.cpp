// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "plugin_api/PluginLoader.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <SDL3/SDL_loadso.h>
#endif

#include <algorithm>
#include <cctype>
#include <cstring>

#include "core/Log.hpp"
#include "core/Sha256.hpp"
#include "core/Version.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::plugin {
namespace {

struct PluginContext {
    std::string name;
    vfs::VirtualFileSystem* vfs = nullptr;
};

spdlog::level::level_enum toLogLevel(sim_log_level level) {
    switch (level) {
    case SIM_LOG_TRACE:
        return spdlog::level::trace;
    case SIM_LOG_DEBUG:
        return spdlog::level::debug;
    case SIM_LOG_INFO:
        return spdlog::level::info;
    case SIM_LOG_WARN:
        return spdlog::level::warn;
    case SIM_LOG_ERROR:
        return spdlog::level::err;
    case SIM_LOG_CRITICAL:
        return spdlog::level::critical;
    }
    return spdlog::level::info;
}

void pluginLog(void* userData, sim_log_level level, const char* message) {
    auto* context = static_cast<PluginContext*>(userData);
    if (context == nullptr || message == nullptr) {
        return;
    }
    core::Log::forPlugin(context->name)->log(toLogLevel(level), "{}", message);
}

std::int64_t pluginVfsRead(void* userData, const char* path, void* buffer, std::size_t bufferSize) {
    auto* context = static_cast<PluginContext*>(userData);
    if (context == nullptr || path == nullptr || buffer == nullptr) {
        return -1;
    }
    const auto data = context->vfs->readBinary(path);
    if (!data || data->empty()) {
        return -1;
    }
    const std::size_t count = std::min(bufferSize, data->size());
    std::memcpy(buffer, data->data(), count);
    return static_cast<std::int64_t>(count);
}

const char* pluginName(void* userData) {
    auto* context = static_cast<PluginContext*>(userData);
    return context != nullptr ? context->name.c_str() : "";
}

// --- Platform loading -------------------------------------------------------
//
// On Windows the plugin is loaded with an explicit search mode so that neither
// the current directory nor PATH can satisfy a missing dependency: the plugin's
// own directory is searched first (LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR) and the
// process default is narrowed to the safe directories. Elsewhere SDL's
// dlopen wrapper is enough.

void* openLibrary(const std::filesystem::path& path, std::string& error) {
#if defined(_WIN32)
    static const bool defaultDirectoriesNarrowed = [] {
        return SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS) != 0;
    }();
    (void)defaultDirectoriesNarrowed;
    const DWORD flags = LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS;
    HMODULE handle = LoadLibraryExW(path.wstring().c_str(), nullptr, flags);
    if (handle == nullptr) {
        error = "LoadLibraryExW failed with error " + std::to_string(static_cast<unsigned long>(GetLastError()));
        return nullptr;
    }
    return handle;
#else
    SDL_SharedObject* handle = SDL_LoadObject(path.string().c_str());
    if (handle == nullptr) {
        error = std::string("cannot open library: ") + SDL_GetError();
    }
    return handle;
#endif
}

void* loadSymbolRaw(void* library, const char* name) {
#if defined(_WIN32)
    return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(library), name));
#else
    return SDL_LoadFunction(static_cast<SDL_SharedObject*>(library), name);
#endif
}

void closeLibrary(void* library) {
    if (library == nullptr) {
        return;
    }
#if defined(_WIN32)
    FreeLibrary(static_cast<HMODULE>(library));
#else
    SDL_UnloadObject(static_cast<SDL_SharedObject*>(library));
#endif
}

template<class Fn>
Fn loadSymbol(void* library, const char* name) {
    void* raw = loadSymbolRaw(library, name);
    Fn function = nullptr;
    static_assert(sizeof(function) == sizeof(raw), "function pointers and data pointers differ in size");
    std::memcpy(&function, &raw, sizeof(function));
    return function;
}

bool hasLibraryExtension(const std::filesystem::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext == ".dll" || ext == ".so" || ext == ".dylib";
}

std::string lowercase(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

// An allowlist entry may be the full file name ("example_native.dll") or just
// the stem ("example_native"); both are compared case-insensitively.
bool isAllowed(const std::filesystem::path& library, const std::vector<std::string>& allow) {
    const std::string file = lowercase(library.filename().string());
    const std::string stem = lowercase(library.stem().string());
    for (const std::string& entry : allow) {
        const std::string normalized = lowercase(std::filesystem::path(entry).filename().string());
        if (normalized == file || normalized == stem) {
            return true;
        }
    }
    return false;
}

} // namespace

PluginLoader::PluginLoader(vfs::VirtualFileSystem& vfs) : vfs_(vfs) {}

PluginLoader::~PluginLoader() {
    unloadAll();
}

bool PluginLoader::load(const std::filesystem::path& libraryPath) {
    PluginRecord record;
    record.path = libraryPath;
    record.name = libraryPath.stem().string();

    const auto fail = [&](std::string message) {
        record.error = std::move(message);
        record.state = PluginState::Failed;
        core::Log::forPlugin(record.name)
            ->error("failed to load '{}' (sha256 {}): {}", libraryPath.filename().string(), record.sha256,
                    record.error);
        plugins_.push_back(std::move(record));
        return false;
    };

    std::string hashError;
    record.sha256 = core::sha256File(libraryPath, hashError);
    if (!hashError.empty()) {
        return fail("cannot fingerprint: " + hashError);
    }

    std::string openError;
    void* library = openLibrary(libraryPath, openError);
    if (library == nullptr) {
        return fail(openError);
    }

    const auto abiVersionFn = loadSymbol<std::uint32_t (*)()>(library, SIM_PLUGIN_ABI_VERSION_SYMBOL);
    if (abiVersionFn == nullptr) {
        closeLibrary(library);
        return fail(std::string("missing symbol ") + SIM_PLUGIN_ABI_VERSION_SYMBOL);
    }

    record.abiVersion = abiVersionFn();
    if (record.abiVersion != SIM_PLUGIN_ABI_VERSION) {
        closeLibrary(library);
        return fail("ABI mismatch: plugin is v" + std::to_string(record.abiVersion) + ", host is v" +
                    std::to_string(SIM_PLUGIN_ABI_VERSION));
    }

    const auto entry = loadSymbol<sim_plugin_entry_fn>(library, SIM_PLUGIN_ENTRY_SYMBOL);
    if (entry == nullptr) {
        closeLibrary(library);
        return fail(std::string("missing symbol ") + SIM_PLUGIN_ENTRY_SYMBOL);
    }

    auto context = std::make_shared<PluginContext>();
    context->name = record.name;
    context->vfs = &vfs_;
    record.context = context;

    sim_plugin_api api{};
    api.abi_version = SIM_PLUGIN_ABI_VERSION;
    api.struct_size = sizeof(sim_plugin_api);
    api.user_data = context.get();
    api.game_version = core::kVersionString;
    api.log = &pluginLog;
    api.vfs_read = &pluginVfsRead;
    api.plugin_name = &pluginName;

    const int result = entry(SIM_PLUGIN_ABI_VERSION, &api, &record.host);
    if (result != 0) {
        closeLibrary(library);
        return fail("entry point returned " + std::to_string(result));
    }

    record.library = library;
    record.entry = entry;
    record.state = PluginState::Loaded;
    core::Log::forPlugin(record.name)
        ->info("loaded native plugin '{}' (ABI v{}, sha256 {})", libraryPath.filename().string(), record.abiVersion,
               record.sha256);
    plugins_.push_back(std::move(record));
    return true;
}

void PluginLoader::loadDirectory(const std::filesystem::path& directory, const PluginLoadOptions& options) {
    enabled_ = options.enabled;

    std::error_code ec;
    if (!std::filesystem::is_directory(directory, ec)) {
        return;
    }

    std::vector<std::filesystem::path> candidates;
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        if (entry.is_regular_file(ec) && hasLibraryExtension(entry.path())) {
            candidates.push_back(entry.path());
        }
    }
    std::sort(candidates.begin(), candidates.end());

    if (!options.enabled) {
        if (!candidates.empty()) {
            core::Log::get("plugin")->warn("native plugins are disabled: {} library/libraries in '{}' were not loaded "
                                           "(set plugins.enabled = true and list each file in plugins.allow)",
                                           candidates.size(), directory.string());
        }
        for (const std::filesystem::path& path : candidates) {
            PluginRecord record;
            record.path = path;
            record.name = path.stem().string();
            std::string hashError;
            record.sha256 = core::sha256File(path, hashError);
            record.state = PluginState::Skipped;
            record.error = "not enabled (plugins.enabled is false)";
            core::Log::get("plugin")->info("skipping '{}' (sha256 {}): plugins.enabled is false",
                                           path.filename().string(), record.sha256);
            plugins_.push_back(std::move(record));
        }
        return;
    }

    for (const std::filesystem::path& path : candidates) {
        if (!isAllowed(path, options.allow)) {
            PluginRecord record;
            record.path = path;
            record.name = path.stem().string();
            std::string hashError;
            record.sha256 = core::sha256File(path, hashError);
            record.state = PluginState::Skipped;
            record.error = "not on the plugins.allow list";
            core::Log::get("plugin")->warn("not loading '{}' (sha256 {}): not listed in plugins.allow",
                                           path.filename().string(), record.sha256);
            plugins_.push_back(std::move(record));
            continue;
        }
        load(path);
    }
}

void PluginLoader::unloadAll() {
    for (PluginRecord& record : plugins_) {
        if (record.state != PluginState::Loaded) {
            continue;
        }
        if (record.host.on_shutdown != nullptr) {
            record.host.on_shutdown(record.host.user_data);
        }
        closeLibrary(record.library);
        record.library = nullptr;
        record.state = PluginState::Skipped;
    }
    plugins_.clear();
}

void PluginLoader::update(double deltaSeconds) {
    for (PluginRecord& record : plugins_) {
        if (record.state == PluginState::Loaded && record.host.on_update != nullptr) {
            record.host.on_update(record.host.user_data, deltaSeconds);
        }
    }
}

bool PluginLoader::hasSkipped() const {
    return std::any_of(plugins_.begin(), plugins_.end(),
                       [](const PluginRecord& record) { return record.state == PluginState::Skipped; });
}

} // namespace sim::plugin
