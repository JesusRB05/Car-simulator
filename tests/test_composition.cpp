// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "modding/Composition.hpp"
#include "vfs/VirtualFileSystem.hpp"

using namespace sim::modding;

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

ContentOwnerFn ownerReturning(const std::string& id) {
    return [id](std::string_view) { return id; };
}

// Both arguments are made explicit so the toml++ overload set is unambiguous.
toml::table parseTable(std::string_view text, std::string_view source) {
    return toml::parse(text, source);
}

} // namespace

TEST_CASE("composition: a patch sets, adds, multiplies and removes dotted keys") {
    std::string error;
    const auto patch = parsePatchText(R"(
target = "base.core/voyager"
[[patch]]
path = "aero.lift_coefficient"
op = "multiply"
value = 1.2
[[patch]]
path = "chassis.mass"
op = "add"
value = 50.0
[[patch]]
path = "suspension.spring_stiffness_front"
op = "set"
value = 45000.0
[[patch]]
path = "engine.limiter"
op = "remove"
)",
                                      "patches/tune.toml", "ana.tune", error);
    REQUIRE(patch.has_value());
    CHECK(error.empty());
    CHECK(patch->modId == "ana.tune");
    CHECK(patch->target == "base.core/voyager");
    REQUIRE(patch->entries.size() == 4);

    toml::table table = parseTable(R"(
[aero]
lift_coefficient = 0.5
[chassis]
mass = 1000.0
[engine]
limiter = 7000.0
)",
                                   "base.core/voyager");

    std::vector<std::string> errors;
    applyPatches({&*patch}, table, &errors);
    CHECK(errors.empty());
    CHECK(table.at_path("aero.lift_coefficient").value<double>() == doctest::Approx(0.6));
    CHECK(table.at_path("chassis.mass").value<double>() == doctest::Approx(1050.0));
    // 'set' creates the intermediate tables it needs.
    CHECK(table.at_path("suspension.spring_stiffness_front").value<double>() == doctest::Approx(45000.0));
    CHECK(table.at_path("engine.limiter").node() == nullptr);
}

TEST_CASE("composition: a bad patch entry is reported but does not stop the rest") {
    std::string error;
    const auto patch = parsePatchText(R"(
target = "base.core/voyager"
[[patch]]
path = "chassis.mass"
op = "add"
value = 10.0
[[patch]]
path = "chassis.does_not_exist"
op = "add"
value = 1.0
)",
                                      "patches/mix.toml", "ana.tune", error);
    REQUIRE(patch.has_value());

    toml::table table = parseTable("[chassis]\nmass = 1000.0\n", "base.core/voyager");
    std::vector<std::string> errors;
    applyPatches({&*patch}, table, &errors);
    REQUIRE(errors.size() == 1);
    CHECK(errors[0].find("does_not_exist") != std::string::npos);
    CHECK(table.at_path("chassis.mass").value<double>() == doctest::Approx(1010.0));
}

TEST_CASE("composition: a malformed patch is rejected with a reason") {
    std::string error;
    CHECK_FALSE(parsePatchText("[[patch]]\npath = \"a\"\nop = \"set\"\nvalue = 1.0\n", "p", "m", error).has_value());
    CHECK(error.find("target") != std::string::npos);

    error.clear();
    CHECK_FALSE(
        parsePatchText("target = \"nope\"\n[[patch]]\npath = \"a\"\nop = \"set\"\nvalue = 1.0\n", "p", "m", error)
            .has_value());
    CHECK(error.find("target") != std::string::npos);

    error.clear();
    CHECK_FALSE(parsePatchText("target = \"base.core/voyager\"\n[[patch]]\npath = \"a\"\nop = \"scale\"\nvalue = 1.0\n",
                               "p", "m", error)
                    .has_value());
    CHECK(error.find("op") != std::string::npos);

    error.clear();
    CHECK_FALSE(
        parsePatchText("target = \"base.core/voyager\"\n[[patch]]\npath = \"a.b\"\nop = \"add\"\n", "p", "m", error)
            .has_value());
    CHECK(error.find("value") != std::string::npos);

    error.clear();
    CHECK_FALSE(parsePatchText("target = \"base.core/voyager\"\n[[patch]]\npath = \"a b\"\nop = \"set\"\nvalue = 1.0\n",
                               "p", "m", error)
                    .has_value());
    CHECK(error.find("path") != std::string::npos);

    error.clear();
    CHECK_FALSE(parsePatchText("target = \"base.core/voyager\"\n", "p", "m", error).has_value());
    CHECK(error.find("patch") != std::string::npos);
}

TEST_CASE("composition: two mods patching the same key are a conflict") {
    Composition composition;
    std::string error;
    for (const std::string& mod : {"ana.a", "bob.b"}) {
        const auto patch = parsePatchText(
            "target = \"base.core/voyager\"\n[[patch]]\npath = \"aero.lift_coefficient\"\nop = \"set\"\nvalue = 1.0\n",
            "patches/lift.toml", mod, error);
        REQUIRE(patch.has_value());
        composition.patches.push_back(*patch);
    }
    detectConflicts(composition, {});
    REQUIRE(composition.conflicts.size() == 1);
    CHECK(composition.conflicts[0].kind == "patch");
    CHECK(composition.conflicts[0].subject == "base.core/voyager (aero.lift_coefficient)");
    CHECK(composition.conflicts[0].mods.size() == 2);

    // The same mod twice is not a conflict with itself.
    Composition solo;
    for (int i = 0; i < 2; ++i) {
        const auto patch = parsePatchText(
            "target = \"base.core/voyager\"\n[[patch]]\npath = \"aero.lift_coefficient\"\nop = \"set\"\nvalue = 1.0\n",
            "patches/lift.toml", "ana.a", error);
        solo.patches.push_back(*patch);
    }
    detectConflicts(solo, {});
    CHECK(solo.conflicts.empty());
}

TEST_CASE("composition: a mod overriding a file another mod overrides is a conflict") {
    Composition composition;
    detectConflicts(
        composition,
        {{"ana.a", "cars/voyager/car.toml"}, {"bob.b", "cars/voyager/car.toml"}, {"carl.c", "tracks/loop/track.toml"}});
    REQUIRE(composition.conflicts.size() == 1);
    CHECK(composition.conflicts[0].kind == "override");
    CHECK(composition.conflicts[0].subject == "cars/voyager/car.toml");
}

TEST_CASE("composition: patches, skins and parts are discovered through the VFS") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_composition";
    fs::remove_all(root);
    writeFile(root / "patches" / "tune.toml",
              "target = \"base.core/voyager\"\n[[patch]]\npath = \"aero.lift_coefficient\"\nop = \"set\"\n"
              "value = 1.0\n");
    writeFile(root / "skins" / "base.core" / "voyager" / "rojo" / "livery.toml",
              "id = \"rojo\"\nname = \"Rojo\"\ncar = \"base.core/voyager\"\nbody_color = [0.75, 0.10, 0.10]\n");
    writeFile(root / "parts" / "base.core" / "voyager" / "rear_wing" / "gt_wing.toml",
              "id = \"gt_wing\"\nname = \"GT wing\"\nslot = \"rear_wing\"\ndownforce_scale = 1.6\n");

    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(root));
    const Composition composition = loadComposition(vfs, ownerReturning("ana.aero"));

    for (const std::string& error : composition.errors) {
        MESSAGE("composition error: " << error);
    }
    REQUIRE(composition.errors.empty());

    REQUIRE(composition.patches.size() == 1);
    CHECK(composition.patches[0].modId == "ana.aero");
    CHECK(composition.patches[0].target == "base.core/voyager");

    REQUIRE(composition.skins.size() == 1);
    CHECK(composition.skins[0].ref == "base.core/voyager/rojo");
    CHECK(composition.skins[0].car == "base.core/voyager");
    CHECK(composition.skins[0].modId == "ana.aero");
    REQUIRE(composition.skins[0].bodyColor.size() == 3);

    REQUIRE(composition.parts.size() == 1);
    CHECK(composition.parts[0].ref == "base.core/voyager/rear_wing/gt_wing");
    CHECK(composition.parts[0].car == "base.core/voyager");
    CHECK(composition.parts[0].slot == "rear_wing");
    REQUIRE(composition.parts[0].values.size() == 1);
    CHECK(composition.parts[0].values[0].first == "downforce_scale");
    CHECK(composition.parts[0].values[0].second == doctest::Approx(1.6));

    CHECK(composition.patchesFor("base.core/voyager").size() == 1);
    CHECK(composition.patchesFor("base.core/other").empty());

    fs::remove_all(root);
}

TEST_CASE("composition: a skin filed under the wrong car is rejected") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_composition_bad";
    fs::remove_all(root);
    writeFile(root / "skins" / "base.core" / "voyager" / "rojo" / "livery.toml",
              "id = \"rojo\"\ncar = \"base.core/other\"\n");

    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(root));
    const Composition composition = loadComposition(vfs, ownerReturning("ana.aero"));
    REQUIRE(composition.skins.empty());
    REQUIRE(composition.errors.size() == 1);
    CHECK(composition.errors[0].find("car") != std::string::npos);

    fs::remove_all(root);
}
