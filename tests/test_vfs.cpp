// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "vfs/VirtualFileSystem.hpp"

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& contents) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << contents;
}

} // namespace

TEST_CASE("VFS caps the size of a single read against zip bombs") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_vfs_max_read";
    fs::remove_all(root);
    writeFile(root / "small.bin", std::string(100, 's'));
    writeFile(root / "big.bin", std::string(4096, 'b'));

    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(root));

    // The default ceiling is generous but finite.
    CHECK(vfs.maxReadSize() == sim::vfs::VirtualFileSystem::kDefaultMaxReadSize);
    CHECK(vfs.maxReadSize() == 256ull * 1024ull * 1024ull);

    // A file within the ceiling reads normally.
    const auto small = vfs.readBinary("small.bin");
    REQUIRE(small.has_value());
    CHECK(small->size() == 100);
    CHECK(vfs.readText("small.bin") == std::optional<std::string>(std::string(100, 's')));

    // Shrink the ceiling below the file: the read is refused, not truncated.
    vfs.setMaxReadSize(1024);
    CHECK_FALSE(vfs.readBinary("big.bin").has_value());
    CHECK_FALSE(vfs.readText("big.bin").has_value());

    // Files below the ceiling are unaffected.
    CHECK(vfs.readText("small.bin").has_value());

    // Raising the ceiling lets the same file through again.
    vfs.setMaxReadSize(8 * 1024);
    const auto big = vfs.readBinary("big.bin");
    REQUIRE(big.has_value());
    CHECK(big->size() == 4096);

    std::error_code ignored;
    fs::remove_all(root, ignored);
}
