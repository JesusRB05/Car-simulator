// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "ModCli.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>
#include <vector>

#include "Paths.hpp"
#include "core/Config.hpp"
#include "core/Log.hpp"
#include "modding/Catalog.hpp"
#include "modding/Manifest.hpp"
#include "modding/ModInstaller.hpp"
#include "modding/Package.hpp"
#include "net/HttpClient.hpp"
#include "platform/Sdl.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

namespace fs = std::filesystem;

// The mods already on disk, by reading their manifests: the same set the app
// would load, including mods that were copied in by hand rather than installed.
std::vector<modding::InstalledMod> scanInstalled(const fs::path& modsDirectory) {
    std::vector<modding::InstalledMod> installed;
    std::error_code ec;
    if (!fs::is_directory(modsDirectory, ec)) {
        return installed;
    }
    for (const fs::directory_iterator::value_type& entry : fs::directory_iterator(modsDirectory, ec)) {
        const std::string name = entry.path().filename().string();
        if (name.starts_with('.') || !entry.is_directory(ec) || modding::hasPackageExtension(entry.path())) {
            continue;
        }
        std::ifstream in(entry.path() / modding::kManifestName, std::ios::binary);
        if (!in) {
            continue;
        }
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const modding::ManifestResult result = modding::parseManifest(text, name);
        if (result.ok()) {
            installed.push_back({result.manifest->id, result.manifest->version.toString()});
        }
    }
    return installed;
}

int checkUpdates(const std::string& indexUrl, const fs::path& modsDirectory) {
    const std::vector<modding::InstalledMod> installed = scanInstalled(modsDirectory);
    const std::unique_ptr<net::HttpClient> http = net::makeHttpClient();
    const modding::Catalog catalog = modding::loadCatalog(*http, indexUrl);
    if (!catalog.ok()) {
        for (const std::string& error : catalog.errors) {
            std::fprintf(stderr, "%s\n", error.c_str());
        }
        return 2;
    }
    for (const std::string& warning : catalog.warnings) {
        std::fprintf(stderr, "warning: %s\n", warning.c_str());
    }

    const std::vector<modding::CatalogUpdate> updates = modding::catalogUpdates(catalog, installed);
    if (updates.empty()) {
        std::printf("no updates: %zu mod(s) installed, %zu id(s) in the index\n", installed.size(),
                    catalog.ids().size());
        return 0;
    }
    for (const modding::CatalogUpdate& update : updates) {
        std::printf("%s %s -> %s\n", update.available.id.c_str(), update.installedVersion.c_str(),
                    update.available.version.c_str());
        if (!update.available.changelog.empty()) {
            std::printf("    %s\n", update.available.changelog.c_str());
        }
        std::printf("    %s\n", update.available.downloadUrl.c_str());
    }
    return 0;
}

} // namespace

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
    const std::filesystem::path modsDirectory = locateModsDirectory(config);
    modding::ModInstaller installer(vfs, modsDirectory);

    if (!options.updatesIndex.empty()) {
        return checkUpdates(options.updatesIndex, modsDirectory);
    }

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
