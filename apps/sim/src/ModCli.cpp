// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "ModCli.hpp"

#include <cstdio>
#include <filesystem>

#include "Paths.hpp"
#include "core/Config.hpp"
#include "core/Log.hpp"
#include "modding/ModInstaller.hpp"
#include "platform/Sdl.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {

int runModCommand(const ModCommandOptions& options) {
    // SDL initialises the platform bits SDL_GetBasePath() needs; no window and
    // no Vulkan device are created for a mod command.
    const platform::SdlContext sdl;

    const std::filesystem::path root = locateRootDirectory();

    core::Log::Config logConfig;
    logConfig.appName = "sim";
    logConfig.logFile = root / "sim.log";
    core::Log::init(logConfig);
    auto log = core::Log::get();

    core::Config config(root / "config.toml");
    config.load();

    vfs::VirtualFileSystem vfs;
    vfs.setWriteDirectory(root / "userdata");

    // The exact directory the app installs and scans, so a mod put here by the
    // CLI is the same one the app picks up.
    modding::ModInstaller installer(vfs, locateModsDirectory(config));

    if (options.install.empty()) {
        std::fprintf(stderr, "no mod command given\n");
        return 2;
    }

    const modding::InstallPlan plan = installer.inspect(options.install);
    std::printf("%s\n", plan.summary().c_str());
    if (!plan.ok) {
        return 2;
    }
    if (options.dryRun) {
        return 0;
    }

    const modding::InstallResult result = installer.install(plan);
    if (!result.ok) {
        log->error("install failed: {}", result.error);
        std::fprintf(stderr, "install failed: %s\n", result.error.c_str());
        return 1;
    }
    std::printf("installed %s\n", result.installed.string().c_str());
    return 0;
}

} // namespace sim::app
