// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <doctest/doctest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "modding/ModInstaller.hpp"
#include "vfs/VirtualFileSystem.hpp"

using namespace sim::modding;

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

std::string readFile(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::string manifest(const std::string& id, const std::string& name, const std::string& version) {
    return "format = 2\nid = \"" + id + "\"\nname = \"" + name + "\"\nversion = \"" + version +
           "\"\nauthors = [\"Test\"]\nlicense = \"GPL-3.0-or-later\"\ntypes = [\"car\"]\n";
}

// --- A minimal stored (uncompressed) zip writer -------------------------------
// The installer reads `.simmod`/`.zip` archives through PhysicsFS, so the test
// needs to build one. Only the two methods the format allows here (stored) are
// written, which keeps the writer short enough to audit.

std::uint32_t crc32(std::string_view data) {
    static std::uint32_t table[256];
    static bool ready = false;
    if (!ready) {
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t value = i;
            for (int bit = 0; bit < 8; ++bit) {
                value = (value & 1u) != 0 ? 0xEDB88320u ^ (value >> 1) : (value >> 1);
            }
            table[i] = value;
        }
        ready = true;
    }
    std::uint32_t crc = 0xFFFFFFFFu;
    for (const char ch : data) {
        crc = table[(crc ^ static_cast<std::uint8_t>(ch)) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

void put16(std::string& out, std::uint16_t value) {
    out.push_back(static_cast<char>(value & 0xFFu));
    out.push_back(static_cast<char>((value >> 8) & 0xFFu));
}

void put32(std::string& out, std::uint32_t value) {
    put16(out, static_cast<std::uint16_t>(value & 0xFFFFu));
    put16(out, static_cast<std::uint16_t>((value >> 16) & 0xFFFFu));
}

struct ZipEntry {
    std::string name;
    std::string data;
};

std::string makeZip(const std::vector<ZipEntry>& entries) {
    struct Central {
        std::uint32_t crc = 0;
        std::uint32_t size = 0;
        std::uint32_t offset = 0;
        std::string name;
    };

    std::string out;
    std::vector<Central> central;
    for (const ZipEntry& entry : entries) {
        Central record;
        record.crc = crc32(entry.data);
        record.size = static_cast<std::uint32_t>(entry.data.size());
        record.offset = static_cast<std::uint32_t>(out.size());
        record.name = entry.name;
        central.push_back(record);

        put32(out, 0x04034b50u);
        put16(out, 20);
        put16(out, 0);
        put16(out, 0);
        put16(out, 0);
        put16(out, 0);
        put32(out, record.crc);
        put32(out, record.size);
        put32(out, record.size);
        put16(out, static_cast<std::uint16_t>(entry.name.size()));
        put16(out, 0);
        out += entry.name;
        out += entry.data;
    }

    const std::uint32_t centralOffset = static_cast<std::uint32_t>(out.size());
    for (const Central& record : central) {
        put32(out, 0x02014b50u);
        put16(out, 20);
        put16(out, 20);
        put16(out, 0);
        put16(out, 0);
        put16(out, 0);
        put16(out, 0);
        put32(out, record.crc);
        put32(out, record.size);
        put32(out, record.size);
        put16(out, static_cast<std::uint16_t>(record.name.size()));
        put16(out, 0);
        put16(out, 0);
        put16(out, 0);
        put16(out, 0);
        put32(out, 0);
        put32(out, record.offset);
        out += record.name;
    }
    const std::uint32_t centralSize = static_cast<std::uint32_t>(out.size()) - centralOffset;

    put32(out, 0x06054b50u);
    put16(out, 0);
    put16(out, 0);
    put16(out, static_cast<std::uint16_t>(central.size()));
    put16(out, static_cast<std::uint16_t>(central.size()));
    put32(out, centralSize);
    put32(out, centralOffset);
    put16(out, 0);
    return out;
}

} // namespace

TEST_CASE("modinstaller: a folder package is inspected and installed") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_installer_folder";
    fs::remove_all(root);
    writeFile(root / "source" / "mod.toml", manifest("ana.karts", "Karts", "1.0.0"));
    writeFile(root / "source" / "cars" / "kart" / "car.toml", "id = \"kart\"\n");
    writeFile(root / "source" / "README.txt", "hello\n");

    sim::vfs::VirtualFileSystem vfs;
    ModInstaller installer(vfs, root / "mods");
    const InstallPlan plan = installer.inspect(root / "source");
    for (const std::string& error : plan.errors) {
        MESSAGE("plan error: " << error);
    }
    REQUIRE(plan.ok);
    CHECK(plan.source == InstallSource::Folder);
    CHECK(plan.modId == "ana.karts");
    CHECK(plan.version == "1.0.0");
    CHECK(plan.target == root / "mods" / "ana.karts");
    CHECK_FALSE(plan.replacesExisting);

    const InstallResult result = installer.install(plan);
    REQUIRE(result.ok);
    CHECK(result.installed == root / "mods" / "ana.karts");
    CHECK(fs::exists(root / "mods" / "ana.karts" / "mod.toml"));
    CHECK(fs::exists(root / "mods" / "ana.karts" / "cars" / "kart" / "car.toml"));
    CHECK(fs::exists(root / "mods" / "ana.karts" / "README.txt"));
    // The staging folder never survives a successful install.
    CHECK_FALSE(fs::exists(root / "mods" / ModInstaller::kStagingName / "ana.karts"));

    fs::remove_all(root);
}

TEST_CASE("modinstaller: a .simmod archive is inspected and installed") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_installer_archive";
    fs::remove_all(root);
    // A GitHub-style download wraps everything in one folder.
    const std::string archive = makeZip({{"my_mod-1.0/mod.toml", manifest("bob.karts", "Bob's Karts", "2.3.0")},
                                         {"my_mod-1.0/cars/kart/car.toml", "id = \"kart\"\n"},
                                         {"my_mod-1.0/README.txt", "thanks\n"}});
    writeFile(root / "bob.simmod", archive);

    sim::vfs::VirtualFileSystem vfs;
    ModInstaller installer(vfs, root / "mods");
    const InstallPlan plan = installer.inspect(root / "bob.simmod");
    for (const std::string& error : plan.errors) {
        MESSAGE("plan error: " << error);
    }
    REQUIRE(plan.ok);
    CHECK(plan.source == InstallSource::Package);
    CHECK(plan.archive);
    CHECK(plan.modId == "bob.karts");
    CHECK(plan.version == "2.3.0");
    // The wrapper folder is reported, not silently swallowed.
    CHECK_FALSE(plan.warnings.empty());

    const InstallResult result = installer.install(plan);
    REQUIRE(result.ok);
    CHECK(fs::exists(root / "mods" / "bob.karts" / "mod.toml"));
    CHECK(fs::exists(root / "mods" / "bob.karts" / "cars" / "kart" / "car.toml"));
    CHECK(fs::exists(root / "mods" / "bob.karts" / "README.txt"));
    // The extraction folder is cleaned up.
    CHECK_FALSE(fs::exists(plan.temp));

    fs::remove_all(root);
}

TEST_CASE("modinstaller: reinstalling keeps a backup that can be rolled back") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_installer_backup";
    fs::remove_all(root);

    sim::vfs::VirtualFileSystem vfs;
    ModInstaller installer(vfs, root / "mods");

    writeFile(root / "v1" / "mod.toml", manifest("ana.karts", "Karts", "1.0.0"));
    writeFile(root / "v1" / "marker.txt", "one\n");
    REQUIRE(installer.install(installer.inspect(root / "v1")).ok);
    CHECK(readFile(root / "mods" / "ana.karts" / "marker.txt") == "one\n");

    writeFile(root / "v2" / "mod.toml", manifest("ana.karts", "Karts", "2.0.0"));
    writeFile(root / "v2" / "marker.txt", "two\n");
    const InstallPlan update = installer.inspect(root / "v2");
    REQUIRE(update.ok);
    CHECK(update.replacesExisting);
    CHECK(update.existingVersion == "1.0.0");

    const InstallResult result = installer.install(update);
    REQUIRE(result.ok);
    CHECK_FALSE(result.backup.empty());
    CHECK(readFile(root / "mods" / "ana.karts" / "marker.txt") == "two\n");

    const InstallResult restore = installer.rollback("ana.karts");
    REQUIRE(restore.ok);
    CHECK(readFile(root / "mods" / "ana.karts" / "marker.txt") == "one\n");

    fs::remove_all(root);
}

TEST_CASE("modinstaller: remove moves a mod out of the mods directory") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_installer_remove";
    fs::remove_all(root);

    sim::vfs::VirtualFileSystem vfs;
    ModInstaller installer(vfs, root / "mods");
    writeFile(root / "source" / "mod.toml", manifest("ana.karts", "Karts", "1.0.0"));
    REQUIRE(installer.install(installer.inspect(root / "source")).ok);

    const InstallResult removed = installer.remove("ana.karts");
    REQUIRE(removed.ok);
    CHECK_FALSE(fs::exists(root / "mods" / "ana.karts"));
    CHECK(installer.remove("ana.karts").ok == false); // already gone

    fs::remove_all(root);
}

TEST_CASE("modinstaller: an uninstallable source is rejected with a reason") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_installer_bad";
    fs::remove_all(root);

    sim::vfs::VirtualFileSystem vfs;
    ModInstaller installer(vfs, root / "mods");

    // No manifest anywhere.
    writeFile(root / "empty" / "readme.txt", "nothing\n");
    const InstallPlan noManifest = installer.inspect(root / "empty");
    CHECK_FALSE(noManifest.ok);
    CHECK_FALSE(noManifest.errors.empty());

    // A manifest that does not name a namespaced id.
    writeFile(root / "bad" / "mod.toml",
              "format = 2\nid = \"NoNamespace\"\nname = \"Bad\"\nversion = \"1.0.0\"\nauthors = [\"Test\"]\n"
              "license = \"GPL-3.0-or-later\"\n");
    const InstallPlan badId = installer.inspect(root / "bad");
    CHECK_FALSE(badId.ok);
    CHECK_FALSE(badId.errors.empty());

    // A path that is neither a folder nor an archive.
    writeFile(root / "notes.txt", "hello\n");
    const InstallPlan unknown = installer.inspect(root / "notes.txt");
    CHECK_FALSE(unknown.ok);
    CHECK(unknown.source == InstallSource::Unknown);

    // Nothing was created by inspecting.
    CHECK_FALSE(fs::exists(root / "mods"));

    fs::remove_all(root);
}
