// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "Paths.hpp"

#include <string>
#include <vector>

#include <SDL3/SDL.h>

#include "core/Config.hpp"

namespace sim::app {
namespace {

constexpr int kMaxRootDepth = 8;

} // namespace

std::filesystem::path executableDirectory() {
    const char* base = SDL_GetBasePath();
    if (base == nullptr) {
        return std::filesystem::current_path();
    }
    // SDL3 owns and caches this string: freeing it would leave later calls
    // returning freed memory.
    return std::filesystem::path(base);
}

std::filesystem::path locateRootDirectory() {
    std::vector<std::filesystem::path> starts{std::filesystem::current_path(), executableDirectory()};
    for (const std::filesystem::path& start : starts) {
        std::filesystem::path dir = start;
        for (int depth = 0; depth < kMaxRootDepth && !dir.empty(); ++depth) {
            if (std::filesystem::exists(dir / "content" / "base")) {
                return dir;
            }
            const std::filesystem::path parent = dir.parent_path();
            if (parent == dir) {
                break;
            }
            dir = parent;
        }
    }
    return executableDirectory();
}

std::filesystem::path userDataDirectory() {
    // SDL owns the platform convention. An empty organisation keeps the path
    // one level deep (`%APPDATA%/Car Simulator/`); if that is rejected, fall
    // back to the org+app form rather than to nothing.
    char* pref = SDL_GetPrefPath("", "Car Simulator");
    if (pref == nullptr) {
        pref = SDL_GetPrefPath("Car Simulator", "Car Simulator");
    }
    if (pref == nullptr) {
        return std::filesystem::current_path() / "userdata";
    }
    const std::filesystem::path directory(pref);
    SDL_free(pref);
    return directory;
}

std::filesystem::path locateModsDirectory(const core::Config& config) {
    const std::string configured = config.getString("mods.directory");
    if (!configured.empty()) {
        return configured;
    }
    return userDataDirectory() / "mods";
}

} // namespace sim::app
