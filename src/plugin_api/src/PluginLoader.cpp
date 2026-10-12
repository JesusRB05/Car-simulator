// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "plugin_api/PluginLoader.hpp"

#include <SDL3/SDL_loadso.h>

#include <algorithm>
#include <cctype>
#include <cstring>

#include "core/Log.hpp"
#include "core/Version.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::plugin {
namespace {

struct PluginContext
{
    std::string name;
    vfs::VirtualFileSystem* vfs = nullptr;
};

spdlog::level::level_enum toLogLevel(sim_log_level level)
{
    switch (level)
    {
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

void pluginLog(void* userData, sim_log_level level, const char* message)
{
    auto* context = static_cast<PluginContext*>(userData);
    if (context == nullptr || message == nullptr)
    {
        return;
    }
    core::Log::forPlugin(context->name)->log(toLogLevel(level), "{}", message);
}

std::int64_t pluginVfsRead(void* userData, const char* path, void* buffer, std::size_t bufferSize)
{
    auto* context = static_cast<PluginContext*>(userData);
    if (context == nullptr || path == nullptr || buffer == nullptr)
    {
        return -1;
    }
    const auto data = context->vfs->readBinary(path);
    if (!data || data->empty())
    {
        return -1;
    }
    const std::size_t count = std::min(bufferSize, data->size());
    std::memcpy(buffer, data->data(), count);
    return static_cast<std::int64_t>(count);
}

const char* pluginName(void* userData)
{
    auto* context = static_cast<PluginContext*>(userData);
    return context != nullptr ? context->name.c_str() : "";
}

template <class Fn>
Fn loadSymbol(SDL_SharedObject* library, const char* name)
{
    void* raw = SDL_LoadFunction(library, name);
    Fn function = nullptr;
    static_assert(sizeof(function) == sizeof(raw), "function pointers and data pointers differ in size");
    std::memcpy(&function, &raw, sizeof(function));
    return function;
}

bool hasLibraryExtension(const std::filesystem::path& path)
{
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext == ".dll" || ext == ".so" || ext == ".dylib";
}

} // namespace

PluginLoader::PluginLoader(vfs::VirtualFileSystem& vfs) : vfs_(vfs) {}

PluginLoader::~PluginLoader()
{
    unloadAll();
}

bool PluginLoader::load(const std::filesystem::path& libraryPath)
{
    PluginRecord record;
    record.path = libraryPath;
    record.name = libraryPath.stem().string();

    const auto fail = [&](std::string message) {
        record.error = std::move(message);
        core::Log::get("plugin")->error("failed to load '{}': {}", libraryPath.string(), record.error);
        plugins_.push_back(std::move(record));
        return false;
    };

    SDL_SharedObject* library = SDL_LoadObject(libraryPath.string().c_str());
    if (library == nullptr)
    {
        return fail(std::string("cannot open library: ") + SDL_GetError());
    }

    const auto abiVersionFn = loadSymbol<std::uint32_t (*)()>(library, SIM_PLUGIN_ABI_VERSION_SYMBOL);
    if (abiVersionFn == nullptr)
    {
        SDL_UnloadObject(library);
        return fail(std::string("missing symbol ") + SIM_PLUGIN_ABI_VERSION_SYMBOL);
    }

    record.abiVersion = abiVersionFn();
    if (record.abiVersion != SIM_PLUGIN_ABI_VERSION)
    {
        SDL_UnloadObject(library);
        return fail("ABI mismatch: plugin is v" + std::to_string(record.abiVersion) + ", host is v" +
                    std::to_string(SIM_PLUGIN_ABI_VERSION));
    }

    const auto entry = loadSymbol<sim_plugin_entry_fn>(library, SIM_PLUGIN_ENTRY_SYMBOL);
    if (entry == nullptr)
    {
        SDL_UnloadObject(library);
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
    if (result != 0)
    {
        SDL_UnloadObject(library);
        return fail("entry point returned " + std::to_string(result));
    }

    record.library = library;
    record.entry = entry;
    record.loaded = true;
    core::Log::forPlugin(record.name)->info("loaded native plugin (ABI v{})", record.abiVersion);
    plugins_.push_back(std::move(record));
    return true;
}

void PluginLoader::loadDirectory(const std::filesystem::path& directory)
{
    std::error_code ec;
    if (!std::filesystem::is_directory(directory, ec))
    {
        return;
    }
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec))
    {
        if (entry.is_regular_file() && hasLibraryExtension(entry.path()))
        {
            load(entry.path());
        }
    }
}

void PluginLoader::unloadAll()
{
    for (PluginRecord& record : plugins_)
    {
        if (record.loaded && record.host.on_shutdown != nullptr)
        {
            record.host.on_shutdown(record.host.user_data);
        }
        if (record.library != nullptr)
        {
            SDL_UnloadObject(static_cast<SDL_SharedObject*>(record.library));
            record.library = nullptr;
        }
        record.loaded = false;
    }
    plugins_.clear();
}

void PluginLoader::update(double deltaSeconds)
{
    for (PluginRecord& record : plugins_)
    {
        if (record.loaded && record.host.on_update != nullptr)
        {
            record.host.on_update(record.host.user_data, deltaSeconds);
        }
    }
}

} // namespace sim::plugin
