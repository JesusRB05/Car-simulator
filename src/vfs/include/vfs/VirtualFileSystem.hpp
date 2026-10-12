// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sim::vfs {

// Virtual file system built on PhysicsFS. Official content (/content/base) and
// every mod are mounted onto a single virtual tree. Later mounts take priority,
// which is what lets a mod overwrite an official file.
//
// All game content (tracks, cars, textures, shaders, scripts) must be read
// through this class. The only files touched outside the VFS are the user
// settings file and the log file, which are owned by the application layer.
//
// PhysicsFS keeps global state and is not thread-safe: a single instance is
// expected, created after the platform layer and used from the main thread.
class VirtualFileSystem
{
public:
    VirtualFileSystem();
    ~VirtualFileSystem();

    VirtualFileSystem(const VirtualFileSystem&) = delete;
    VirtualFileSystem& operator=(const VirtualFileSystem&) = delete;

    // Mounts a directory or an archive (.zip) at the given virtual mount point.
    // When append is false (default) the new mount takes priority over previous
    // ones; when true it is appended below them.
    bool mount(const std::filesystem::path& source, std::string_view mountPoint = "/", bool append = false);

    // Unmounts a previously mounted source. Returns true if it was mounted.
    bool unmount(const std::filesystem::path& source);
    void unmountAll();

    void setWriteDirectory(const std::filesystem::path& dir);
    std::filesystem::path writeDirectory() const;

    bool exists(std::string_view virtualPath) const;
    bool isDirectory(std::string_view virtualPath) const;

    std::optional<std::vector<std::uint8_t>> readBinary(std::string_view virtualPath) const;
    std::optional<std::string> readText(std::string_view virtualPath) const;

    // Writes into the write directory (creating parent directories as needed).
    bool writeText(std::string_view virtualPath, std::string_view contents);

    std::vector<std::string> list(std::string_view virtualDir) const;

    // Physical location (real directory) that provided a given virtual file.
    // Returns an empty string if the file is not found.
    std::string realDirectoryOf(std::string_view virtualPath) const;

    // All currently mounted sources, most recently mounted first.
    std::vector<std::string> searchPaths() const;

    std::string lastError() const;

private:
    bool initialised_ = false;
};

} // namespace sim::vfs
