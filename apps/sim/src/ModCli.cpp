// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
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
#include "modding/Fingerprint.hpp"
#include "modding/Manifest.hpp"
#include "modding/ModInstaller.hpp"
#include "modding/Package.hpp"
#include "net/HttpClient.hpp"
#include "platform/Sdl.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

namespace fs = std::filesystem;

// Every mod on disk, by reading its manifest: the same set the app would load,
// including mods that were copied in by hand rather than installed.
std::vector<modding::ContentSource> scanInstalled(const fs::path& modsDirectory) {
    std::vector<modding::ContentSource> installed;
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
            installed.push_back(
                {result.manifest->id, result.manifest->version.toString(), result.manifest->types, entry.path()});
        }
    }
    return installed;
}

std::vector<modding::InstalledMod> installedFrom(const std::vector<modding::ContentSource>& sources) {
    std::vector<modding::InstalledMod> installed;
    installed.reserve(sources.size());
    for (const modding::ContentSource& source : sources) {
        installed.push_back({source.id, source.version});
    }
    return installed;
}

int checkUpdates(const std::string& indexUrl, const std::vector<modding::ContentSource>& sources) {
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

    const std::vector<modding::CatalogUpdate> updates = modding::catalogUpdates(catalog, installedFrom(sources));
    if (updates.empty()) {
        std::printf("no updates: %zu mod(s) installed, %zu id(s) in the index\n", sources.size(), catalog.ids().size());
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

// Prints what a peer would have to have, in the same shape the check reads.
int printFingerprint(const std::vector<modding::ContentSource>& sources) {
    const modding::ContentFingerprint fingerprint = modding::fingerprintOf(sources);
    std::fputs(modding::writeRequiredContent(fingerprint).c_str(), stdout);
    return 0;
}

// Compares a session's required content with what is installed here. This is
// the check a client runs before it joins, and it is what `--mod-fingerprint`
// output is for.
int checkContent(const std::string& file, const std::vector<modding::ContentSource>& sources) {
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot read '%s'\n", file.c_str());
        return 2;
    }
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const modding::RequiredContentResult required = modding::parseRequiredContent(text, file);
    if (!required.ok()) {
        for (const std::string& error : required.errors) {
            std::fprintf(stderr, "%s\n", error.c_str());
        }
        return 2;
    }

    const modding::ContentComparison comparison =
        modding::compareContent(required.fingerprint.entries, modding::fingerprintOf(sources));
    std::printf("%s\n", comparison.summary().c_str());
    for (const modding::ContentMismatch& mismatch : comparison.mismatches) {
        std::printf("  ! %s\n", mismatch.message.c_str());
        if (!mismatch.required.url.empty()) {
            std::printf("    %s\n", mismatch.required.url.c_str());
        }
    }
    for (const std::string& extra : comparison.extra) {
        std::printf("  + extra local mod: %s\n", extra.c_str());
    }
    return comparison.matches() ? 0 : 1;
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
    // Diagnostics go to the log file, not to the console: the output of these
    // commands is meant to be read by a person or piped somewhere (a
    // fingerprinted session file, a list of updates), and a log line at the top
    // would break it.
    logConfig.console = false;
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
        return checkUpdates(options.updatesIndex, scanInstalled(modsDirectory));
    }
    if (options.fingerprint) {
        // The CLI fingerprints what is installed; the app fingerprints what is
        // active. Both go through the same comparison.
        return printFingerprint(scanInstalled(modsDirectory));
    }
    if (!options.contentCheck.empty()) {
        return checkContent(options.contentCheck, scanInstalled(modsDirectory));
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
