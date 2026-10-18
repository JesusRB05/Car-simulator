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

// A cheap fingerprint of a source tree: the name, size and write time of every
// file, mixed into one number. Enough to notice that a developer saved a file
// (mtime), added or removed one (count), without hashing the bytes.
std::uint64_t sourceStamp(const std::filesystem::path& source) {
    std::uint64_t stamp = 1469598103934665603ull; // FNV-1a offset basis
    const auto mix = [&stamp](std::uint64_t value) {
        stamp ^= value;
        stamp *= 1099511628211ull;
    };

    std::error_code ec;
    if (std::filesystem::is_directory(source, ec)) {
        std::vector<std::filesystem::path> files;
        for (auto it = std::filesystem::recursive_directory_iterator(source, ec);
             !ec && it != std::filesystem::recursive_directory_iterator(); it.increment(ec)) {
            std::error_code fileEc;
            if (it->is_regular_file(fileEc)) {
                files.push_back(it->path());
            }
        }
        std::sort(files.begin(), files.end());
        for (const std::filesystem::path& file : files) {
            std::error_code fileEc;
            const auto time = std::filesystem::last_write_time(file, fileEc);
            mix(static_cast<std::uint64_t>(time.time_since_epoch().count()));
            mix(static_cast<std::uint64_t>(std::filesystem::file_size(file, fileEc)));
        }
        mix(static_cast<std::uint64_t>(files.size()));
    } else {
        const auto time = std::filesystem::last_write_time(source, ec);
        mix(static_cast<std::uint64_t>(time.time_since_epoch().count()));
        mix(static_cast<std::uint64_t>(std::filesystem::file_size(source, ec)));
    }
    return stamp;
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
    : vfs_(vfs), config_(config), paths_(std::move(paths)), profiles_(paths_.profilesFile) {
    if (!paths_.profilesFile.empty()) {
        std::string error;
        if (!profiles_.load(&error) && !error.empty()) {
            core::Log::get("modding")->warn("cannot read profiles: {}", error);
        }
    }
}

void ModManager::reload() {
    activeIds_ = config_.getStringArray(kActiveKey);
    discover();
    mountActive();
    if (developer_) {
        refreshWatch();
    }
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
    std::vector<std::filesystem::path> modDirectories = paths_.extraModDirectories;
    if (!paths_.modsDirectory.empty()) {
        modDirectories.push_back(paths_.modsDirectory);
    }
    for (const std::filesystem::path& directory : modDirectories) {
        if (!std::filesystem::is_directory(directory, ec)) {
            continue;
        }
        std::vector<std::filesystem::path> entries;
        for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
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
                    discovered.status.overrides = result.manifest->overrides;
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
    // The requested set lives in the config, which reload() reads back: writing
    // it there (not just in memory) is what makes the change survive reload().
    std::vector<std::string> ids = config_.getStringArray(kActiveKey);
    const auto it = std::find(ids.begin(), ids.end(), id);
    if (active) {
        if (it == ids.end()) {
            ids.emplace_back(id);
        }
    } else if (it != ids.end()) {
        ids.erase(it);
    }
    config_.setStringArray(kActiveKey, ids);
    config_.save();
    reload();
    return true;
}

bool ModManager::applyProfile(std::string_view name) {
    const std::optional<Profile> profile = profiles_.find(name);
    if (!profile) {
        return false;
    }
    config_.setStringArray(kActiveKey, profile->mods);
    config_.save();
    profiles_.setLastApplied(profile->name);
    std::string error;
    if (!profiles_.save(&error)) {
        core::Log::get("modding")->warn("cannot save profiles: {}", error);
    }
    reload();
    return true;
}

bool ModManager::saveProfile(std::string_view name) {
    if (name.empty()) {
        return false;
    }
    if (!profiles_.set(Profile{std::string(name), config_.getStringArray(kActiveKey)})) {
        return false;
    }
    std::string error;
    if (!profiles_.save(&error)) {
        core::Log::get("modding")->warn("cannot save profiles: {}", error);
        return false;
    }
    return true;
}

bool ModManager::deleteProfile(std::string_view name) {
    if (!profiles_.remove(name)) {
        return false;
    }
    std::string error;
    if (!profiles_.save(&error)) {
        core::Log::get("modding")->warn("cannot save profiles: {}", error);
        return false;
    }
    return true;
}

std::vector<ContentSource> ModManager::contentSources() const {
    std::vector<ContentSource> sources;
    for (const ModStatus& mod : report_.mods) {
        if (mod.state != ModState::Active) {
            continue;
        }
        sources.push_back({mod.id, mod.version, mod.types, mod.source});
    }
    return sources;
}

ContentFingerprint ModManager::contentFingerprint() const {
    return fingerprintOf(contentSources());
}

void ModManager::setDeveloper(bool enabled) {
    developer_ = enabled;
    if (developer_) {
        refreshWatch();
    } else {
        watch_.clear();
    }
}

void ModManager::refreshWatch() {
    watch_.clear();
    watch_.reserve(discovered_.size());
    for (const Discovered& discovered : discovered_) {
        watch_.emplace_back(discovered.status.source, sourceStamp(discovered.status.source));
    }
}

bool ModManager::pollDevelopment() {
    if (!developer_) {
        return false;
    }

    bool changed = false;
    std::vector<std::pair<std::filesystem::path, std::uint64_t>> next;
    next.reserve(discovered_.size());
    for (const Discovered& discovered : discovered_) {
        const std::uint64_t stamp = sourceStamp(discovered.status.source);
        next.emplace_back(discovered.status.source, stamp);
        const auto previous = std::find_if(watch_.begin(), watch_.end(), [&discovered](const auto& entry) {
            return entry.first == discovered.status.source;
        });
        // A source we have never seen (a mod dropped into the folder) counts as
        // a change; so does a different stamp on a source we already watch.
        if (previous == watch_.end() || previous->second != stamp) {
            changed = true;
        }
    }
    if (next.size() != watch_.size()) {
        changed = true;
    }
    watch_ = std::move(next);
    if (changed) {
        core::Log::get("modding")->info("developer mode: a mod source changed");
    }
    return changed;
}

std::string ModManager::modIdFor(std::string_view virtualPath) const {
    const std::string real = vfs_.realDirectoryOf(virtualPath);
    if (real.empty()) {
        return {};
    }
    std::error_code ec;
    const std::filesystem::path canonical = std::filesystem::weakly_canonical(real, ec);
    if (ec) {
        return {};
    }
    for (const Discovered& discovered : discovered_) {
        std::error_code sourceEc;
        if (std::filesystem::weakly_canonical(discovered.status.source, sourceEc) == canonical) {
            return discovered.manifest ? discovered.manifest->id : std::string{};
        }
    }
    return {};
}

} // namespace sim::modding
