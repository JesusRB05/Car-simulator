// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/ModInstaller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <sstream>
#include <system_error>
#include <utility>

#include "core/Log.hpp"
#include "modding/Collection.hpp"
#include "modding/Manifest.hpp"
#include "modding/Package.hpp"
#include "modding/Registry.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::modding {
namespace {

namespace fs = std::filesystem;

std::string lowerExtension(const fs::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

bool isArchive(const fs::path& path) {
    const std::string ext = lowerExtension(path);
    return ext == ".zip" || ext == kPackageExtension;
}

std::string uniqueSuffix() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::to_string(now);
}

// Descends while a folder has no mod.toml but holds exactly one subfolder. A
// GitHub export wraps a package in `repo-branch/`, and some authors zip the
// folder itself; both are recognised and the wasted level is reported.
fs::path findPackageRoot(const fs::path& start, std::vector<std::string>& warnings) {
    fs::path current = start;
    fs::path wrapped; // the folder we had to descend into, if any
    for (int depth = 0; depth < 4; ++depth) {
        std::error_code ec;
        if (fs::exists(current / kManifestName, ec)) {
            if (!wrapped.empty()) {
                warnings.push_back("the manifest lives inside '" + wrapped.filename().string() +
                                   "'; that subfolder became the package root");
            }
            return current;
        }
        fs::path onlyFolder;
        bool severalFolders = false;
        int looseFiles = 0;
        for (const fs::directory_iterator::value_type& entry : fs::directory_iterator(current, ec)) {
            const std::string name = entry.path().filename().string();
            if (name.starts_with('.')) {
                continue; // .DS_Store, __MACOSX and friends
            }
            if (fs::is_directory(entry.path(), ec)) {
                if (onlyFolder.empty()) {
                    onlyFolder = entry.path();
                } else {
                    severalFolders = true;
                }
            } else {
                ++looseFiles;
            }
        }
        if (severalFolders || onlyFolder.empty()) {
            break;
        }
        if (looseFiles > 0) {
            warnings.push_back("ignoring " + std::to_string(looseFiles) + " file(s) beside the mod folder");
        }
        current = onlyFolder;
        if (wrapped.empty()) {
            wrapped = current;
        }
    }
    std::error_code ec;
    return fs::exists(current / kManifestName, ec) ? current : fs::path{};
}

// Copies every file of a mounted archive out to `destination`. Every entry name
// is validated first (no absolute paths, no `..`, no drive letters) and the
// whole extraction is bounded, so a crafted archive cannot escape the
// destination directory, fill the disk or exhaust memory.
bool extractArchive(vfs::VirtualFileSystem& vfs, const fs::path& asZip, const fs::path& destination,
                    std::vector<std::string>& errors) {
    const std::string mountPoint = "/__modinstall";
    if (!vfs.mount(asZip, mountPoint, false)) {
        errors.push_back("cannot open the archive: " + vfs.lastError());
        return false;
    }
    std::vector<std::pair<std::string, fs::path>> pending{{mountPoint, destination}};
    bool ok = true;
    std::size_t entries = 0;
    std::uint64_t totalBytes = 0;
    while (!pending.empty() && ok) {
        auto [virtualDir, realDir] = pending.back();
        pending.pop_back();
        std::error_code ec;
        fs::create_directories(realDir, ec);
        for (const std::string& name : vfs.list(virtualDir)) {
            const std::string child = virtualDir + "/" + name;
            const std::string relative = child.substr(mountPoint.size() + 1);
            if (!isSafePackagePath(relative)) {
                errors.push_back("the archive contains an unsafe path: '" + relative + "'");
                ok = false;
                break;
            }
            if (isNativeBinaryPath(relative)) {
                errors.push_back("the archive ships '" + relative +
                                 "'; native binaries are only allowed inside plugins/");
                ok = false;
                break;
            }
            if (vfs.isDirectory(child)) {
                pending.emplace_back(child, realDir / name);
                continue;
            }
            if (++entries > kMaxPackageEntries) {
                errors.push_back("the archive has more than " + std::to_string(kMaxPackageEntries) + " files");
                ok = false;
                break;
            }
            const auto bytes = vfs.readBinary(child);
            if (!bytes) {
                errors.push_back("cannot read '" + relative + "' from the archive");
                ok = false;
                break;
            }
            totalBytes += bytes->size();
            if (totalBytes > kMaxPackageBytes) {
                errors.push_back("the archive is larger than " + std::to_string(kMaxPackageBytes) + " bytes");
                ok = false;
                break;
            }
            std::ofstream out(realDir / name, std::ios::binary);
            out.write(reinterpret_cast<const char*>(bytes->data()), static_cast<std::streamsize>(bytes->size()));
            if (!out) {
                errors.push_back("cannot write '" + (realDir / name).string() + "'");
                ok = false;
                break;
            }
        }
    }
    vfs.unmount(asZip);
    return ok;
}

// A folder source bypasses the archive walk, so the same rules are applied to
// its files directly: no unsafe names and no native binary outside `plugins/`.
bool checkPayloadFiles(const fs::path& payload, std::vector<std::string>& errors) {
    std::error_code ec;
    bool ok = true;
    std::size_t entries = 0;
    std::uint64_t totalBytes = 0;
    for (const fs::recursive_directory_iterator::value_type& entry : fs::recursive_directory_iterator(payload, ec)) {
        const std::string relative = entry.path().lexically_relative(payload).generic_string();
        if (!isSafePackagePath(relative)) {
            errors.push_back("the package contains an unsafe path: '" + relative + "'");
            return false;
        }
        if (!entry.is_regular_file(ec)) {
            continue;
        }
        if (isNativeBinaryPath(relative)) {
            errors.push_back("the package ships '" + relative + "'; native binaries are only allowed inside plugins/");
            ok = false;
        }
        if (++entries > kMaxPackageEntries) {
            errors.push_back("the package has more than " + std::to_string(kMaxPackageEntries) + " files");
            return false;
        }
        totalBytes += static_cast<std::uint64_t>(entry.file_size(ec));
        if (totalBytes > kMaxPackageBytes) {
            errors.push_back("the package is larger than " + std::to_string(kMaxPackageBytes) + " bytes");
            return false;
        }
    }
    return ok;
}

bool readFile(const fs::path& path, std::string& text) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

// The version of a mod already installed at `directory`, or an empty string.
std::string installedVersion(const fs::path& directory) {
    std::string text;
    if (!readFile(directory / kManifestName, text)) {
        return {};
    }
    const ManifestResult result = parseManifest(text, directory.filename().string());
    return result.ok() ? result.manifest->version.toString() : std::string{};
}

} // namespace

std::string InstallPlan::summary() const {
    if (!ok) {
        std::ostringstream oss;
        oss << "cannot install " << input.string();
        for (const std::string& error : errors) {
            oss << "\n  ! " << error;
        }
        return oss.str();
    }

    std::ostringstream oss;
    oss << (replacesExisting ? "Update " : "Install ") << name << " " << version << " (" << modId << ")";
    if (replacesExisting) {
        oss << ", replacing " << (existingVersion.empty() ? "the installed version" : existingVersion);
    }
    oss << "\n  into " << target.string();
    if (!types.empty()) {
        oss << "\n  provides:";
        for (const std::string& type : types) {
            oss << " " << type;
        }
    }
    if (!permissions.empty()) {
        oss << "\n  asks for:";
        for (const std::string& permission : permissions) {
            oss << " " << permission;
        }
    }
    if (collection) {
        oss << "\n  collection of " << collection->entries.size() << " mod(s):";
        for (const CollectionEntry& entry : collection->entries) {
            oss << " " << entry.modId;
            if (entry.optional) {
                oss << "(optional)";
            }
        }
    }
    if (!digest.empty()) {
        oss << "\n  sha256: " << digest;
    }
    for (const std::string& warning : warnings) {
        oss << "\n  ~ " << warning;
    }
    return oss.str();
}

ModInstaller::ModInstaller(vfs::VirtualFileSystem& vfs, std::filesystem::path modsDirectory)
    : vfs_(vfs), mods_(std::move(modsDirectory)) {}

InstallPlan ModInstaller::inspect(const std::filesystem::path& source) const {
    InstallPlan plan;
    plan.input = source;

    std::error_code ec;
    if (!fs::exists(source, ec)) {
        plan.errors.push_back("'" + source.string() + "' does not exist");
        return plan;
    }

    if (fs::is_directory(source, ec)) {
        plan.source = InstallSource::Folder;
        plan.payload = findPackageRoot(source, plan.warnings);
        if (plan.payload.empty()) {
            plan.errors.push_back("no " + std::string(kManifestName) + " found in '" + source.string() + "'");
            return plan;
        }
        if (!checkPayloadFiles(plan.payload, plan.errors)) {
            return plan;
        }
    } else if (isArchive(source)) {
        plan.source = InstallSource::Package;
        plan.archive = true;
        plan.temp = fs::temp_directory_path() / ("simmod-" + uniqueSuffix());
        fs::create_directories(plan.temp, ec);
        if (ec) {
            plan.errors.push_back("cannot create a working folder: " + ec.message());
            return plan;
        }
        // PhysicsFS chooses the archiver by extension, so a `.simmod` is staged
        // under a `.zip` name before it is mounted.
        const fs::path asZip = plan.temp / "package.zip";
        fs::copy_file(source, asZip, ec);
        if (ec) {
            plan.errors.push_back("cannot read '" + source.string() + "': " + ec.message());
            return plan;
        }
        const fs::path extracted = plan.temp / "extracted";
        if (!extractArchive(vfs_, asZip, extracted, plan.errors)) {
            return plan;
        }
        plan.payload = findPackageRoot(extracted, plan.warnings);
        if (plan.payload.empty()) {
            plan.errors.push_back("no " + std::string(kManifestName) + " found in '" + source.filename().string() +
                                  "'");
            return plan;
        }
    } else {
        plan.source = InstallSource::Unknown;
        plan.errors.push_back("'" + source.filename().string() + "' is neither a folder nor a " +
                              std::string(kPackageExtension) + " file");
        return plan;
    }

    std::string text;
    if (!readFile(plan.payload / kManifestName, text)) {
        plan.errors.push_back("cannot read '" + (plan.payload / kManifestName).string() + "'");
        return plan;
    }
    const ManifestResult result = parseManifest(text, plan.payload.filename().string());
    plan.warnings.insert(plan.warnings.end(), result.warnings.begin(), result.warnings.end());
    if (!result.ok()) {
        plan.errors.insert(plan.errors.end(), result.errors.begin(), result.errors.end());
        return plan;
    }

    plan.modId = result.manifest->id;
    plan.name = result.manifest->name;
    plan.version = result.manifest->version.toString();
    plan.types = result.manifest->types;
    plan.permissions = result.manifest->permissions;
    plan.digest = payloadDigest(plan.payload);
    plan.target = mods_ / plan.modId;

    // A collection is a normal mod that also lists other mods to bring along.
    std::string collectionText;
    if (readFile(plan.payload / kCollectionName, collectionText)) {
        const CollectionResult parsed = parseCollection(collectionText, plan.payload.filename().string());
        plan.warnings.insert(plan.warnings.end(), parsed.warnings.begin(), parsed.warnings.end());
        if (parsed.ok()) {
            plan.collection = parsed.collection;
            const bool declaresCollection = std::any_of(plan.types.begin(), plan.types.end(),
                                                        [](const std::string& type) { return type == "collection"; });
            if (!declaresCollection) {
                plan.warnings.push_back("the package lists mods in " + std::string(kCollectionName) +
                                        " but does not declare 'collection' in types");
            }
        } else {
            plan.errors.insert(plan.errors.end(), parsed.errors.begin(), parsed.errors.end());
            return plan;
        }
    }
    if (fs::exists(plan.target, ec)) {
        plan.replacesExisting = true;
        plan.existingVersion = installedVersion(plan.target);
    }

    plan.ok = plan.errors.empty();
    if (plan.ok) {
        core::Log::get("modding")->info("install plan: {}", plan.summary());
    } else {
        for (const std::string& error : plan.errors) {
            core::Log::get("modding")->warn("install plan: {}", error);
        }
    }
    return plan;
}

InstallResult ModInstaller::install(const InstallPlan& plan) {
    InstallResult result;
    if (!plan.ok) {
        result.error = "the plan is not valid and cannot be installed";
        return result;
    }

    std::error_code ec;
    const fs::path staging = mods_ / kStagingName / plan.modId;
    fs::remove_all(staging, ec);
    fs::create_directories(staging, ec);
    if (ec) {
        result.error = "cannot create '" + staging.string() + "': " + ec.message();
        return result;
    }
    fs::copy(plan.payload, staging, fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
    if (ec) {
        fs::remove_all(staging, ec);
        result.error = "cannot write '" + staging.string() + "': " + ec.message();
        return result;
    }

    const fs::path target = mods_ / plan.modId;
    fs::path backup;
    if (fs::exists(target, ec)) {
        backup = mods_ / kBackupName / plan.modId;
        fs::remove_all(backup, ec);
        fs::create_directories(backup.parent_path(), ec);
        fs::rename(target, backup, ec);
        if (ec) {
            fs::remove_all(staging, ec);
            result.error = "cannot move the installed '" + target.string() + "' aside: " + ec.message();
            return result;
        }
    }

    fs::rename(staging, target, ec);
    if (ec) {
        std::error_code restoreEc;
        if (!backup.empty()) {
            fs::rename(backup, target, restoreEc);
        }
        std::error_code cleanupEc;
        fs::remove_all(staging, cleanupEc);
        result.error = "cannot install into '" + target.string() + "': " + ec.message();
        return result;
    }

    if (plan.archive && !plan.temp.empty()) {
        std::error_code cleanupEc;
        fs::remove_all(plan.temp, cleanupEc);
    }
    // The staging root only exists to make the move atomic; it is empty now.
    std::error_code pruneEc;
    fs::remove(mods_ / kStagingName, pruneEc);

    // Record what was installed and where it came from: the registry is the
    // only place the origin is remembered, and the digest lets the manager
    // notice a payload that changed behind the installer's back.
    ModRegistry registry(mods_ / kRegistryName);
    registry.load();
    registry.upsert({plan.modId, plan.version, payloadDigest(target), plan.input.string()});
    std::string registryError;
    if (!registry.save(&registryError)) {
        core::Log::get("modding")->warn("cannot update the mod registry: {}", registryError);
    }

    result.ok = true;
    result.installed = target;
    result.backup = backup;
    core::Log::get("modding")->info("installed '{}' into '{}'", plan.modId, target.string());
    return result;
}

InstallResult ModInstaller::remove(std::string_view modId) {
    InstallResult result;
    std::error_code ec;
    const fs::path target = mods_ / std::string(modId);
    if (!fs::exists(target, ec)) {
        result.error = "mod '" + std::string(modId) + "' is not installed";
        return result;
    }
    const fs::path backup = mods_ / kBackupName / std::string(modId);
    fs::remove_all(backup, ec);
    fs::create_directories(backup.parent_path(), ec);
    fs::rename(target, backup, ec);
    if (ec) {
        result.error = "cannot remove '" + target.string() + "': " + ec.message();
        return result;
    }
    ModRegistry registry(mods_ / kRegistryName);
    registry.load();
    registry.remove(modId);
    std::string registryError;
    if (!registry.save(&registryError)) {
        core::Log::get("modding")->warn("cannot update the mod registry: {}", registryError);
    }
    result.ok = true;
    result.installed = target;
    result.backup = backup;
    return result;
}

InstallResult ModInstaller::rollback(std::string_view modId) {
    InstallResult result;
    std::error_code ec;
    const fs::path backup = mods_ / kBackupName / std::string(modId);
    if (!fs::exists(backup, ec)) {
        result.error = "no backup of mod '" + std::string(modId) + "' to restore";
        return result;
    }
    const fs::path target = mods_ / std::string(modId);
    fs::remove_all(target, ec);
    fs::rename(backup, target, ec);
    if (ec) {
        result.error = "cannot restore '" + target.string() + "': " + ec.message();
        return result;
    }
    // Re-derive the record from the restored files, keeping the origin we knew.
    ModRegistry registry(mods_ / kRegistryName);
    registry.load();
    const std::optional<RegistryEntry> previous = registry.find(modId);
    std::string version;
    std::string text;
    if (readFile(target / kManifestName, text)) {
        const ManifestResult manifest = parseManifest(text, target.filename().string());
        if (manifest.ok()) {
            version = manifest.manifest->version.toString();
        }
    }
    registry.upsert({std::string(modId), version, payloadDigest(target), previous ? previous->source : std::string{}});
    std::string registryError;
    if (!registry.save(&registryError)) {
        core::Log::get("modding")->warn("cannot update the mod registry: {}", registryError);
    }
    result.ok = true;
    result.installed = target;
    result.backup = backup;
    return result;
}

} // namespace sim::modding
