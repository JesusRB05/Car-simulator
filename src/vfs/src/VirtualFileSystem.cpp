// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vfs/VirtualFileSystem.hpp"

#include <physfs.h>

#include "core/Log.hpp"

namespace sim::vfs {
namespace {

std::string normalise(std::string_view path)
{
    std::string result(path);
    if (result.empty() || result.front() != '/')
    {
        result.insert(result.begin(), '/');
    }
    return result;
}

void logLastError(std::string_view what)
{
    const PHYSFS_ErrorCode code = PHYSFS_getLastErrorCode();
    core::Log::get("vfs")->error("{}: {}", what, PHYSFS_getErrorByCode(code));
}

} // namespace

VirtualFileSystem::VirtualFileSystem()
{
    if (!PHYSFS_init("car-simulator"))
    {
        logLastError("PHYSFS_init");
        return;
    }
    PHYSFS_permitSymbolicLinks(1);
    initialised_ = true;
}

VirtualFileSystem::~VirtualFileSystem()
{
    if (initialised_)
    {
        PHYSFS_deinit();
        initialised_ = false;
    }
}

bool VirtualFileSystem::mount(const std::filesystem::path& source, std::string_view mountPoint, bool append)
{
    if (!initialised_)
    {
        return false;
    }
    const std::string point = normalise(mountPoint);
    const std::string native = source.string();
    if (!PHYSFS_mount(native.c_str(), point.c_str(), append ? 1 : 0))
    {
        logLastError("PHYSFS_mount '" + native + "'");
        return false;
    }
    core::Log::get("vfs")->debug("mounted '{}' at '{}' (priority {})", native, point, append ? "low" : "high");
    return true;
}

bool VirtualFileSystem::unmount(const std::filesystem::path& source)
{
    if (!initialised_)
    {
        return false;
    }
    const std::string native = source.string();
    if (!PHYSFS_unmount(native.c_str()))
    {
        logLastError("PHYSFS_unmount '" + native + "'");
        return false;
    }
    return true;
}

void VirtualFileSystem::unmountAll()
{
    if (!initialised_)
    {
        return;
    }
    // PhysicsFS has no PHYSFS_unmountAll: enumerate the current search path and
    // unmount each entry (PHYSFS_unmount removes every mount point for a dir).
    char** list = PHYSFS_getSearchPath();
    if (list == nullptr)
    {
        logLastError("PHYSFS_getSearchPath");
        return;
    }
    std::vector<std::string> paths;
    for (char** entry = list; *entry != nullptr; ++entry)
    {
        paths.emplace_back(*entry);
    }
    PHYSFS_freeList(list);
    for (const std::string& path : paths)
    {
        PHYSFS_unmount(path.c_str());
    }
}

void VirtualFileSystem::setWriteDirectory(const std::filesystem::path& dir)
{
    if (!initialised_)
    {
        return;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (!PHYSFS_setWriteDir(dir.string().c_str()))
    {
        logLastError("PHYSFS_setWriteDir");
    }
}

std::filesystem::path VirtualFileSystem::writeDirectory() const
{
    if (!initialised_)
    {
        return {};
    }
    const char* dir = PHYSFS_getWriteDir();
    return dir != nullptr ? std::filesystem::path(dir) : std::filesystem::path{};
}

bool VirtualFileSystem::exists(std::string_view virtualPath) const
{
    if (!initialised_)
    {
        return false;
    }
    const std::string path = normalise(virtualPath);
    return PHYSFS_exists(path.c_str()) != 0;
}

bool VirtualFileSystem::isDirectory(std::string_view virtualPath) const
{
    if (!initialised_)
    {
        return false;
    }
    const std::string path = normalise(virtualPath);
    return PHYSFS_isDirectory(path.c_str()) > 0;
}

std::optional<std::vector<std::uint8_t>> VirtualFileSystem::readBinary(std::string_view virtualPath) const
{
    if (!initialised_)
    {
        return std::nullopt;
    }
    const std::string path = normalise(virtualPath);
    PHYSFS_File* file = PHYSFS_openRead(path.c_str());
    if (file == nullptr)
    {
        logLastError("PHYSFS_openRead '" + path + "'");
        return std::nullopt;
    }

    const PHYSFS_sint64 length = PHYSFS_fileLength(file);
    std::vector<std::uint8_t> buffer(static_cast<std::size_t>(length > 0 ? length : 0));
    const PHYSFS_sint64 read = PHYSFS_readBytes(file, buffer.data(), static_cast<PHYSFS_uint64>(buffer.size()));
    PHYSFS_close(file);

    if (read < 0)
    {
        logLastError("PHYSFS_readBytes '" + path + "'");
        return std::nullopt;
    }
    buffer.resize(static_cast<std::size_t>(read));
    return buffer;
}

std::optional<std::string> VirtualFileSystem::readText(std::string_view virtualPath) const
{
    auto bytes = readBinary(virtualPath);
    if (!bytes)
    {
        return std::nullopt;
    }
    return std::string(reinterpret_cast<const char*>(bytes->data()), bytes->size());
}

bool VirtualFileSystem::writeText(std::string_view virtualPath, std::string_view contents)
{
    if (!initialised_)
    {
        return false;
    }
    const std::string path = normalise(virtualPath);

    // Create parent directories level by level.
    std::size_t slash = path.find('/', 1);
    while (slash != std::string::npos)
    {
        const std::string dir = path.substr(0, slash);
        PHYSFS_mkdir(dir.c_str());
        slash = path.find('/', slash + 1);
    }

    PHYSFS_File* file = PHYSFS_openWrite(path.c_str());
    if (file == nullptr)
    {
        logLastError("PHYSFS_openWrite '" + path + "'");
        return false;
    }
    const PHYSFS_sint64 written =
        PHYSFS_writeBytes(file, contents.data(), static_cast<PHYSFS_uint64>(contents.size()));
    PHYSFS_close(file);
    return written == static_cast<PHYSFS_sint64>(contents.size());
}

std::vector<std::string> VirtualFileSystem::list(std::string_view virtualDir) const
{
    std::vector<std::string> entries;
    if (!initialised_)
    {
        return entries;
    }
    const std::string path = normalise(virtualDir);
    char** listing = PHYSFS_enumerateFiles(path.c_str());
    if (listing == nullptr)
    {
        logLastError("PHYSFS_enumerateFiles '" + path + "'");
        return entries;
    }
    for (char** entry = listing; *entry != nullptr; ++entry)
    {
        entries.emplace_back(*entry);
    }
    PHYSFS_freeList(listing);
    return entries;
}

std::string VirtualFileSystem::realDirectoryOf(std::string_view virtualPath) const
{
    if (!initialised_)
    {
        return {};
    }
    const std::string path = normalise(virtualPath);
    const char* dir = PHYSFS_getRealDir(path.c_str());
    return dir != nullptr ? std::string(dir) : std::string{};
}

std::vector<std::string> VirtualFileSystem::searchPaths() const
{
    std::vector<std::string> paths;
    if (!initialised_)
    {
        return paths;
    }
    char** list = PHYSFS_getSearchPath();
    if (list == nullptr)
    {
        return paths;
    }
    for (char** entry = list; *entry != nullptr; ++entry)
    {
        paths.emplace_back(*entry);
    }
    PHYSFS_freeList(list);
    return paths;
}

std::string VirtualFileSystem::lastError() const
{
    const PHYSFS_ErrorCode code = PHYSFS_getLastErrorCode();
    return PHYSFS_getErrorByCode(code);
}

} // namespace sim::vfs
