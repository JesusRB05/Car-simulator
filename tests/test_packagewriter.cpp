// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "modding/PackageWriter.hpp"
#include "vfs/VirtualFileSystem.hpp"

using namespace sim::modding;
namespace vfs = sim::vfs;

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

// PhysicsFS picks its archiver by extension, so the package (a `.simmod`) is
// mounted through a `.zip` copy, exactly as the installer does.
fs::path mountableCopy(const fs::path& package) {
    const fs::path copy = package.parent_path() / (package.stem().string() + "-mount.zip");
    fs::copy_file(package, copy, fs::copy_options::overwrite_existing);
    return copy;
}

} // namespace

TEST_CASE("package writer: the archive holds the same files, and the VFS reads them") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_packagewriter";
    fs::remove_all(root);

    const std::string binary("\x89PNG\r\n\x1a\n\0\1\2\3", 12);
    writeFile(root / "ana.karts" / "mod.toml", "format = 2\nid = \"ana.karts\"\n");
    writeFile(root / "ana.karts" / "cars" / "kart" / "car.toml", "id = \"kart\"\n");
    writeFile(root / "ana.karts" / "cars" / "kart" / "model" / "car.glb", binary);
    writeFile(root / "ana.karts" / "README.md", "# Kart\n");

    const fs::path package = root / "ana.karts-0.1.0.simmod";
    std::string error;
    REQUIRE(writePackage(root / "ana.karts", package, error));
    if (!error.empty()) {
        MESSAGE("package error: " << error);
    }

    vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(mountableCopy(package), "/"));
    CHECK(vfs.exists("mod.toml"));
    CHECK(vfs.exists("cars/kart/model/car.glb"));
    CHECK(vfs.readText("cars/kart/car.toml").value_or("") == "id = \"kart\"\n");
    CHECK(vfs.readText("README.md").value_or("") == "# Kart\n");
    const auto bytes = vfs.readBinary("cars/kart/model/car.glb");
    REQUIRE(bytes.has_value());
    CHECK(std::string(bytes->begin(), bytes->end()) == binary);

    // The mount keeps its archive open; close it before cleaning up.
    vfs.unmountAll();
    fs::remove_all(root);
}

TEST_CASE("package writer: the same content always produces the same bytes") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_packagewriter_stable";
    fs::remove_all(root);

    // Two copies of the same mod, created in a different order.
    writeFile(root / "one" / "mod.toml", "format = 2\n");
    writeFile(root / "one" / "cars" / "kart" / "car.toml", "id = \"kart\"\n");
    writeFile(root / "one" / "patches" / "a.toml", "target = \"base.core/voyager\"\n");

    writeFile(root / "two" / "patches" / "a.toml", "target = \"base.core/voyager\"\n");
    writeFile(root / "two" / "cars" / "kart" / "car.toml", "id = \"kart\"\n");
    writeFile(root / "two" / "mod.toml", "format = 2\n");

    std::string error;
    REQUIRE(writePackage(root / "one", root / "one.simmod", error));
    REQUIRE(writePackage(root / "two", root / "two.simmod", error));

    // A package is a release artifact: rebuilding it must give the same file,
    // so a digest is worth publishing and two builds can be compared.
    CHECK(readFile(root / "one.simmod") == readFile(root / "two.simmod"));

    fs::remove_all(root);
}

TEST_CASE("package writer: an empty folder is refused with a reason") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_packagewriter_empty";
    fs::remove_all(root);
    fs::create_directories(root / "empty");

    std::string error;
    CHECK_FALSE(writePackage(root / "empty", root / "out.simmod", error));
    CHECK_FALSE(error.empty());
    CHECK_FALSE(writePackage(root / "missing", root / "out.simmod", error));
    CHECK_FALSE(error.empty());

    fs::remove_all(root);
}
