// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "core/Sha256.hpp"
#include "plugin_api/PluginLoader.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& contents) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << contents;
}

const sim::plugin::PluginRecord* findPlugin(const std::vector<sim::plugin::PluginRecord>& records,
                                            const std::string& name) {
    for (const sim::plugin::PluginRecord& record : records) {
        if (record.name == name) {
            return &record;
        }
    }
    return nullptr;
}

} // namespace

TEST_CASE("SHA-256 matches the FIPS 180-4 test vectors") {
    using sim::core::sha256Hex;

    CHECK(sha256Hex("", 0) == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    const std::string abc = "abc";
    CHECK(sha256Hex(abc.data(), abc.size()) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

    // 56 bytes: the padding spills into a second block, exercising the buffer.
    const std::string message = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    CHECK(sha256Hex(message.data(), message.size()) ==
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");

    // One million 'a', fed in chunks so the incremental path is covered.
    sim::core::Sha256 hash;
    const std::string chunk(1000, 'a');
    for (int i = 0; i < 1000; ++i) {
        hash.update(chunk.data(), chunk.size());
    }
    CHECK(hash.hexDigest() == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST_CASE("sha256File hashes a file and reports a missing one") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_sha256";
    fs::remove_all(root);
    writeFile(root / "abc.bin", "abc");

    std::string error;
    CHECK(sim::core::sha256File(root / "abc.bin", error) ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(error.empty());

    error.clear();
    CHECK(sim::core::sha256File(root / "missing.bin", error).empty());
    CHECK_FALSE(error.empty());

    std::error_code ignored;
    fs::remove_all(root, ignored);
}

TEST_CASE("native plugins are opt-in: nothing loads with plugins.enabled off") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_plugins_disabled";
    fs::remove_all(root);
    const fs::path plugins = root / "plugins";
    writeFile(plugins / "trusted.dll", "not a real library");
    writeFile(plugins / "other.dll", "not a real library");

    const std::string expectedHash = sim::core::sha256Hex("not a real library", 18);

    sim::vfs::VirtualFileSystem vfs;
    sim::plugin::PluginLoader loader(vfs);
    sim::plugin::PluginLoadOptions options; // enabled = false by default

    loader.loadDirectory(plugins, options);

    CHECK_FALSE(loader.enabled());
    REQUIRE(loader.plugins().size() == 2);
    CHECK(loader.hasSkipped());
    for (const sim::plugin::PluginRecord& record : loader.plugins()) {
        CHECK(record.state == sim::plugin::PluginState::Skipped);
        CHECK(record.sha256 == expectedHash); // every library is still fingerprinted
    }

    std::error_code ignored;
    fs::remove_all(root, ignored);
}

TEST_CASE("native plugins are gated by the plugins.allow list") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_plugins_allow";
    fs::remove_all(root);
    const fs::path plugins = root / "plugins";
    writeFile(plugins / "trusted.dll", "not a real library");
    writeFile(plugins / "other.dll", "not a real library");

    const std::string expectedHash = sim::core::sha256Hex("not a real library", 18);

    sim::vfs::VirtualFileSystem vfs;
    sim::plugin::PluginLoader loader(vfs);
    sim::plugin::PluginLoadOptions options;
    options.enabled = true;
    options.allow = {"other.dll"}; // trusted.dll is not listed and must be skipped

    loader.loadDirectory(plugins, options);

    CHECK(loader.enabled());
    REQUIRE(loader.plugins().size() == 2);

    const sim::plugin::PluginRecord* skipped = findPlugin(loader.plugins(), "trusted");
    REQUIRE(skipped != nullptr);
    CHECK(skipped->state == sim::plugin::PluginState::Skipped);
    CHECK(skipped->error.find("plugins.allow") != std::string::npos);
    CHECK(skipped->sha256 == expectedHash);

    // The approved file is attempted: it is not a real library, so it fails,
    // but it was fingerprinted and it was not silently ignored.
    const sim::plugin::PluginRecord* attempted = findPlugin(loader.plugins(), "other");
    REQUIRE(attempted != nullptr);
    CHECK(attempted->state == sim::plugin::PluginState::Failed);
    CHECK_FALSE(attempted->error.empty());
    CHECK(attempted->sha256 == expectedHash);

    std::error_code ignored;
    fs::remove_all(root, ignored);
}

TEST_CASE("the plugins.allow list accepts a bare stem") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_plugins_stem";
    fs::remove_all(root);
    const fs::path plugins = root / "plugins";
    writeFile(plugins / "trusted.dll", "not a real library");

    sim::vfs::VirtualFileSystem vfs;
    sim::plugin::PluginLoader loader(vfs);
    sim::plugin::PluginLoadOptions options;
    options.enabled = true;
    options.allow = {"trusted"}; // stem, no extension

    loader.loadDirectory(plugins, options);

    REQUIRE(loader.plugins().size() == 1);
    // Attempted (and failed, since it is not a real library), not skipped.
    CHECK(loader.plugins().front().state == sim::plugin::PluginState::Failed);

    std::error_code ignored;
    fs::remove_all(root, ignored);
}
