// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include "ModCli.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <system_error>
#include <vector>

#include "ModCreator.hpp"
#include "Paths.hpp"
#include "core/Config.hpp"
#include "core/Log.hpp"
#include "modding/Catalog.hpp"
#include "modding/Fingerprint.hpp"
#include "modding/Manifest.hpp"
#include "modding/ModInstaller.hpp"
#include "modding/Package.hpp"
#include "modding/Registry.hpp"
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

void printModUsage() {
    std::printf("sim mod commands (docs/MODS.md section 9)\n"
                "\n"
                "  sim mod new car   ana.karts/kart [--from base.core/voyager] [--out <dir>]\n"
                "  sim mod new track bob.hills/hillclimb\n"
                "  sim mod new skin  carl.liveries/red --for base.core/voyager\n"
                "  sim mod new patch ana.tuning/stiffer --for base.core/voyager\n"
                "  sim mod validate <folder|.simmod>     parse everything the game parses\n"
                "  sim mod pack <folder> [--out <file>]  build a .simmod, digest and index entry\n"
                "  sim mod list                          what is installed, and what is on\n"
                "  sim mod install <folder|.simmod>      the same install the game performs\n"
                "  sim mod remove|enable|disable <id>\n"
                "\n"
                "  sim --install-mod <path> [--dry-run]  install without a window\n"
                "  sim --mod-updates <index>             what in the index is newer\n"
                "  sim --mod-fingerprint                 the content a session needs\n"
                "  sim --content-check <file>            check one against this machine\n");
}

// The install path the app uses, so `sim mod install` and the in-game button
// cannot disagree.
int installOne(modding::ModInstaller& installer, const std::string& source, bool dryRun,
               const std::shared_ptr<spdlog::logger>& log) {
    const modding::InstallPlan plan = installer.inspect(source);
    std::printf("%s\n", plan.summary().c_str());
    if (!plan.ok) {
        return 2;
    }
    if (dryRun) {
        return 0;
    }
    const modding::InstallResult result = installer.install(plan);
    if (!result.ok) {
        log->error("install failed: {}", result.error);
        std::fprintf(stderr, "install failed: %s\n", result.error.c_str());
        return 1;
    }
    std::printf("installed %s\n", result.installed.generic_string().c_str());
    return 0;
}

int listMods(const fs::path& modsDirectory) {
    // `mods.active` is the requested set; the app resolves dependencies from it.
    core::Config config(locateRootDirectory() / "config.toml");
    config.load();
    const std::vector<std::string> requested = config.getStringArray("mods.active");

    const std::vector<modding::ContentSource> sources = scanInstalled(modsDirectory);
    if (sources.empty()) {
        std::printf("no mods in %s\n", modsDirectory.string().c_str());
        return 0;
    }
    std::printf("%-24s %-10s %-9s %s\n", "id", "version", "state", "types");
    for (const modding::ContentSource& source : sources) {
        const bool on = source.id == modding::kBaseModId ||
                        std::find(requested.begin(), requested.end(), source.id) != requested.end();
        std::string types;
        for (const std::string& type : source.types) {
            if (!types.empty()) {
                types += ", ";
            }
            types += type;
        }
        std::printf("%-24s %-10s %-9s %s\n", source.id.c_str(), source.version.c_str(), on ? "active" : "disabled",
                    types.c_str());
    }
    return 0;
}

int removeMod(const std::string& id, const fs::path& modsDirectory) {
    if (id == modding::kBaseModId) {
        std::fprintf(stderr, "the base content is not a mod that can be removed\n");
        return 2;
    }
    if (!modding::isValidModId(id) && !modding::isLegacyModId(id)) {
        std::fprintf(stderr, "'%s' is not a mod id\n", id.c_str());
        return 2;
    }

    std::error_code ec;
    const fs::path directory = modsDirectory / id;
    if (!fs::is_directory(directory, ec)) {
        std::fprintf(stderr, "'%s' is not installed in %s\n", id.c_str(), modsDirectory.string().c_str());
        return 1;
    }
    fs::remove_all(directory, ec);
    if (ec) {
        std::fprintf(stderr, "cannot remove '%s': %s\n", directory.string().c_str(), ec.message().c_str());
        return 1;
    }

    // The registry is the installer's memory of where a mod came from; a mod
    // removed behind its back would linger there and be reported as tampered.
    modding::ModRegistry registry(modsDirectory / std::string(modding::ModInstaller::kRegistryName));
    if (registry.load() && registry.find(id).has_value()) {
        registry.remove(id);
        std::string error;
        if (!registry.save(&error)) {
            std::fprintf(stderr, "warning: %s\n", error.c_str());
        }
    }

    // A mod still in the requested set would be reported as missing on the next
    // start, so it is taken out of it too.
    core::Config config(locateRootDirectory() / "config.toml");
    config.load();
    std::vector<std::string> requested = config.getStringArray("mods.active");
    const auto it = std::remove(requested.begin(), requested.end(), id);
    if (it != requested.end()) {
        requested.erase(it, requested.end());
        config.setStringArray("mods.active", requested);
        config.save();
    }

    std::printf("removed %s\n", id.c_str());
    return 0;
}

int setModEnabled(const std::string& id, bool enabled, const fs::path& modsDirectory) {
    if (id == modding::kBaseModId) {
        std::fprintf(stderr, "the base content is always on\n");
        return 2;
    }
    const std::vector<modding::ContentSource> sources = scanInstalled(modsDirectory);
    const bool installed = std::any_of(sources.begin(), sources.end(),
                                       [&id](const modding::ContentSource& source) { return source.id == id; });
    if (!installed) {
        std::fprintf(stderr, "'%s' is not installed in %s\n", id.c_str(), modsDirectory.string().c_str());
        return 1;
    }

    core::Config config(locateRootDirectory() / "config.toml");
    config.load();
    std::vector<std::string> requested = config.getStringArray("mods.active");
    const auto it = std::find(requested.begin(), requested.end(), id);
    if (enabled && it == requested.end()) {
        requested.emplace_back(id);
    } else if (!enabled && it != requested.end()) {
        requested.erase(it);
    } else {
        std::printf("'%s' is already %s\n", id.c_str(), enabled ? "enabled" : "disabled");
        return 0;
    }
    config.setStringArray("mods.active", requested);
    config.save();
    std::printf("%s %s\n", enabled ? "enabled" : "disabled", id.c_str());
    return 0;
}

int runModSubcommand(const std::vector<std::string>& args, modding::ModInstaller& installer,
                     const fs::path& modsDirectory, const std::shared_ptr<spdlog::logger>& log) {
    const std::string verb = args[0];
    auto value = [&args](std::string_view name) -> std::string {
        for (std::size_t i = 0; i + 1 < args.size(); ++i) {
            if (args[i] == name) {
                return args[i + 1];
            }
        }
        return {};
    };

    if (verb == "help" || verb == "--help" || verb == "-h") {
        printModUsage();
        return 0;
    }
    if (verb == "new") {
        if (args.size() < 3) {
            std::fprintf(stderr,
                         "usage: sim mod new <car|track|skin|patch> <mod-id>/<name> [--from <ref>] [--for <ref>]\n");
            return 2;
        }
        ModNewOptions options;
        options.type = args[1];
        options.target = args[2];
        options.from = value("--from");
        options.forCar = value("--for");
        options.output = value("--out");
        return createMod(options);
    }
    if (verb == "validate") {
        if (args.size() < 2) {
            std::fprintf(stderr, "usage: sim mod validate <folder|.simmod>\n");
            return 2;
        }
        return validateMod(args[1]);
    }
    if (verb == "pack") {
        if (args.size() < 2) {
            std::fprintf(stderr, "usage: sim mod pack <folder> [--out <file>]\n");
            return 2;
        }
        return packMod(args[1], value("--out"));
    }
    if (verb == "list") {
        return listMods(modsDirectory);
    }
    if (verb == "install") {
        if (args.size() < 2) {
            std::fprintf(stderr, "usage: sim mod install <folder|.simmod>\n");
            return 2;
        }
        return installOne(installer, args[1], false, log);
    }
    if (verb == "remove") {
        if (args.size() < 2) {
            std::fprintf(stderr, "usage: sim mod remove <id>\n");
            return 2;
        }
        return removeMod(args[1], modsDirectory);
    }
    if (verb == "enable" || verb == "disable") {
        if (args.size() < 2) {
            std::fprintf(stderr, "usage: sim mod %s <id>\n", verb.c_str());
            return 2;
        }
        return setModEnabled(args[1], verb == "enable", modsDirectory);
    }

    std::fprintf(stderr, "unknown mod command '%s'\n", verb.c_str());
    printModUsage();
    return 2;
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

    // `sim mod <verb> ...` (the creator and manager commands).
    if (!options.args.empty()) {
        return runModSubcommand(options.args, installer, modsDirectory, log);
    }

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
        printModUsage();
        return 2;
    }

    return installOne(installer, options.install, options.dryRun, log);
}

} // namespace sim::app
