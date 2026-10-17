// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/ModManager.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

#include "core/Config.hpp"
#include "core/Log.hpp"
#include "modding/Package.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::modding {
namespace {

constexpr const char* kActiveKey = "mods.active";

std::string lowerExtension(const std::filesystem::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

bool isMountableSource(const std::filesystem::path& path) {
    std::error_code ec;
    if (std::filesystem::is_directory(path, ec)) {
        return true;
    }
    const std::string ext = lowerExtension(path);
    return ext == ".zip" || ext == kPackageExtension;
}

// The content types implied by the folders a package actually ships. Used for a
// v1 manifest and as a fallback when `types` is missing.
struct FolderType {
    std::string_view folder;
    std::string_view type;
};

constexpr FolderType kFolderTypes[] = {
    {kCarsDir, "car"},    {kTracksDir, "track"},    {kSkinsDir, "skins"},     {kPartsDir, "parts"},
    {kTyresDir, "tyres"}, {kSoundsDir, "sounds"},   {kSetupsDir, "setups"},   {kWeatherDir, "weather"},
    {kAppsDir, "apps"},   {kModesDir, "modes"},     {kVisualsDir, "visuals"}, {kLocaleDir, "locale"},
    {kInputDir, "input"}, {kPluginsDir, "plugins"}, {kPatchesDir, "patches"},
};

std::vector<std::string> inferTypes(vfs::VirtualFileSystem& vfs, const std::string& mountPoint) {
    std::vector<std::string> types;
    for (const FolderType& entry : kFolderTypes) {
        const std::string directory = mountPoint + "/" + std::string(entry.folder);
        if (!vfs.list(directory).empty()) {
            types.emplace_back(entry.type);
        }
    }
    return types;
}

} // namespace

bool ModReport::ok() const {
    return std::none_of(mods.begin(), mods.end(), [](const ModStatus& m) { return m.state == ModState::Invalid; });
}

std::string ModReport::toString() const {
    std::ostringstream oss;
    oss << "mod report: " << mods.size() << " mod(s) discovered\n";
    for (const ModStatus& mod : mods) {
        const char* state = "disabled";
        if (mod.state == ModState::Active) {
            state = "active";
        } else if (mod.state == ModState::Invalid) {
            state = "INVALID";
        }
        oss << "  - " << mod.id << " " << mod.version << " [" << state << "]";
        if (!mod.types.empty()) {
            oss << " types:";
            for (const auto& type : mod.types) {
                oss << " " << type;
            }
        }
        if (!mod.dependencies.empty()) {
            oss << " deps:";
            for (const auto& dependency : mod.dependencies) {
                oss << " " << dependency;
            }
        }
        oss << "\n";
        for (const auto& warning : mod.warnings) {
            oss << "      ~ " << warning << "\n";
        }
        for (const auto& error : mod.errors) {
            oss << "      ! " << error << "\n";
        }
    }
    for (const auto& message : messages) {
        oss << "  * " << message << "\n";
    }
    return oss.str();
}

ModManager::ModManager(vfs::VirtualFileSystem& vfs, core::Config& config, Paths paths)
    : vfs_(vfs), config_(config), paths_(std::move(paths)) {}

void ModManager::reload() {
    activeIds_ = config_.getStringArray(kActiveKey);
    discover();
    mountActive();
}

void ModManager::discover() {
    discovered_.clear();

    core::Log::get("modding")->info("scanning mods: base='{}' mods='{}'", paths_.baseContent.string(),
                                    paths_.modsDirectory.string());

    // Collect candidate sources: base content first, then each mod entry.
    struct Candidate {
        std::filesystem::path source;
        bool isBase;
    };
    std::vector<Candidate> candidates;
    candidates.push_back({paths_.baseContent, true});

    std::error_code ec;
    if (std::filesystem::is_directory(paths_.modsDirectory, ec)) {
        std::vector<std::filesystem::path> entries;
        for (const auto& entry : std::filesystem::directory_iterator(paths_.modsDirectory, ec)) {
            if (entry.path().filename().string().starts_with('.')) {
                continue;
            }
            if (isMountableSource(entry.path())) {
                entries.push_back(entry.path());
            }
        }
        std::sort(entries.begin(), entries.end());
        for (const auto& entry : entries) {
            candidates.push_back({entry, false});
        }
    }

    int probeIndex = 0;
    for (const Candidate& candidate : candidates) {
        Discovered discovered;
        discovered.status.source = candidate.source;
        discovered.status.isBase = candidate.isBase;

        const std::string mountPoint = "/__probe/" + std::to_string(probeIndex++);
        const std::string fallbackId = candidate.source.stem().string();

        bool parsed = false;
        if (vfs_.mount(candidate.source, mountPoint, false)) {
            const std::string manifestPath = mountPoint + "/mod.toml";
            if (auto text = vfs_.readText(manifestPath)) {
                ManifestResult result = parseManifest(*text, candidate.source.filename().string());
                if (result.ok()) {
                    discovered.manifest = *result.manifest;
                    discovered.status.id = result.manifest->id;
                    discovered.status.name = result.manifest->name;
                    discovered.status.version = result.manifest->version.toString();
                    discovered.status.types = result.manifest->types;
                    discovered.status.warnings = result.warnings;
                    discovered.status.migratedFromV1 = result.manifest->migratedFromV1;
                    if (discovered.status.types.empty()) {
                        discovered.status.types = inferTypes(vfs_, mountPoint);
                    }
                    discovered.status.dependencies.reserve(result.manifest->dependencies.size());
                    for (const auto& dependency : result.manifest->dependencies) {
                        discovered.status.dependencies.push_back(dependency.id + " " + dependency.versionRange);
                    }
                    parsed = true;
                } else {
                    discovered.status.errors = result.errors;
                    discovered.status.warnings = result.warnings;
                    discovered.status.id = fallbackId;
                    discovered.status.name = fallbackId;
                }
            } else {
                discovered.status.errors.push_back("'" + candidate.source.filename().string() + "' has no mod.toml");
                discovered.status.id = fallbackId;
                discovered.status.name = fallbackId;
            }
            vfs_.unmount(candidate.source);
        } else {
            discovered.status.errors.push_back("cannot mount '" + candidate.source.string() + "'");
            discovered.status.id = fallbackId;
            discovered.status.name = fallbackId;
        }

        if (!parsed) {
            discovered.status.state = ModState::Invalid;
        }
        discovered_.push_back(std::move(discovered));
    }
}

void ModManager::mountActive() {
    // Build the manifest list and the requested set.
    std::vector<ModManifest> manifests;
    for (const Discovered& discovered : discovered_) {
        if (discovered.manifest) {
            manifests.push_back(*discovered.manifest);
        }
    }

    // The base content is always requested.
    std::vector<std::string> requested = activeIds_;
    for (const Discovered& discovered : discovered_) {
        if (discovered.status.isBase && discovered.manifest) {
            if (std::find(requested.begin(), requested.end(), discovered.manifest->id) == requested.end()) {
                requested.push_back(discovered.manifest->id);
            }
        }
    }

    const ResolveResult resolution = resolveMods(manifests, requested);

    // Mount base content first (lowest priority), then active mods by priority.
    vfs_.unmountAll();
    if (!paths_.baseContent.empty()) {
        vfs_.mount(paths_.baseContent, "/", false);
    }

    std::vector<const Discovered*> activeMods;
    for (const std::string& id : resolution.order) {
        for (const Discovered& discovered : discovered_) {
            if (discovered.manifest && discovered.manifest->id == id && !discovered.status.isBase) {
                activeMods.push_back(&discovered);
                break;
            }
        }
    }
    std::stable_sort(activeMods.begin(), activeMods.end(), [](const Discovered* a, const Discovered* b) {
        return a->manifest->priority < b->manifest->priority;
    });
    for (const Discovered* mod : activeMods) {
        vfs_.mount(mod->status.source, "/", false);
    }

    // Build the report.
    report_ = ModReport{};
    for (const Discovered& discovered : discovered_) {
        ModStatus status = discovered.status;
        if (status.state != ModState::Invalid && discovered.manifest) {
            const bool isActive =
                std::find(resolution.order.begin(), resolution.order.end(), status.id) != resolution.order.end();
            status.state = isActive ? ModState::Active : ModState::Disabled;
        }
        report_.mods.push_back(std::move(status));
    }
    report_.messages = resolution.conflicts;
    for (const ResolveError& error : resolution.errors) {
        report_.messages.push_back(error.modId + ": " + error.message);
    }
    for (const auto& cycle : resolution.cycles) {
        std::string text = "cycle:";
        for (const auto& member : cycle) {
            text += " " + member;
        }
        report_.messages.push_back(text);
    }

    // Persist the active list so the user's choice survives restarts.
    std::vector<std::string> persist;
    for (const Discovered& discovered : discovered_) {
        if (!discovered.status.isBase && discovered.manifest &&
            std::find(resolution.order.begin(), resolution.order.end(), discovered.manifest->id) !=
                resolution.order.end() &&
            std::find(activeIds_.begin(), activeIds_.end(), discovered.manifest->id) != activeIds_.end()) {
            persist.push_back(discovered.manifest->id);
        }
    }
    config_.setStringArray(kActiveKey, persist);
    config_.save();

    core::Log::get("modding")->info("{}", report_.toString());
}

bool ModManager::isActive(std::string_view id) const {
    for (const Discovered& discovered : discovered_) {
        if (discovered.manifest && discovered.manifest->id == id) {
            return std::any_of(report_.mods.begin(), report_.mods.end(),
                               [id](const ModStatus& m) { return m.id == id && m.state == ModState::Active; });
        }
    }
    return false;
}

bool ModManager::setModActive(std::string_view id, bool active) {
    const std::string key(id);
    const auto it = std::find(activeIds_.begin(), activeIds_.end(), key);
    if (active) {
        if (it == activeIds_.end()) {
            activeIds_.push_back(key);
        }
    } else if (it != activeIds_.end()) {
        activeIds_.erase(it);
    }
    reload();
    return true;
}

} // namespace sim::modding
