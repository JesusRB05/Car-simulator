// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>

#include "core/Config.hpp"
#include "modding/DependencyResolver.hpp"
#include "modding/Manifest.hpp"
#include "modding/ModManager.hpp"
#include "modding/Package.hpp"
#include "modding/Version.hpp"
#include "vfs/VirtualFileSystem.hpp"

using namespace sim::modding;

TEST_CASE("Version parses and orders semantic versions") {
    const auto version = Version::parse("1.2.3");
    REQUIRE(version.has_value());
    CHECK(version->major() == 1);
    CHECK(version->minor() == 2);
    CHECK(version->patch() == 3);
    CHECK(Version::parse("not-a-version") == std::nullopt);

    CHECK(Version(1, 2, 3) < Version(1, 2, 4));
    CHECK(Version(1, 2, 3) < Version(1, 3, 0));
    CHECK(Version(2, 0, 0) > Version(1, 9, 9));
    CHECK(Version(1, 0, 0) == Version::parse("1.0.0").value());
}

TEST_CASE("Version ranges are satisfied as documented") {
    CHECK(satisfies("*", Version(9, 9, 9)));
    CHECK(satisfies("^1.2.0", Version(1, 5, 0)));
    CHECK_FALSE(satisfies("^1.2.0", Version(2, 0, 0)));
    CHECK(satisfies("~1.2.3", Version(1, 2, 9)));
    CHECK_FALSE(satisfies("~1.2.3", Version(1, 3, 0)));
    CHECK(satisfies(">=1.0.0 <2.0.0", Version(1, 4, 2)));
    CHECK_FALSE(satisfies(">=1.0.0 <2.0.0", Version(2, 0, 0)));
}

TEST_CASE("A valid manifest parses and an invalid one is rejected") {
    const std::string valid = "id = \"my_mod\"\n"
                              "name = \"My Mod\"\n"
                              "version = \"1.0.0\"\n"
                              "author = \"Someone\"\n"
                              "license = \"CC BY-SA 4.0\"\n";
    ManifestResult result = parseManifest(valid, "my_mod");
    REQUIRE(result.ok());
    CHECK(result.manifest->id == "my_mod");
    CHECK(result.manifest->version == Version(1, 0, 0));

    const std::string invalid = "id = \"Invalid Id\"\nname = \"x\"\nversion = \"1.0.0\"\n";
    ManifestResult bad = parseManifest(invalid, "bad");
    CHECK_FALSE(bad.ok());
}

TEST_CASE("Dependencies are ordered before their dependents") {
    ModManifest a;
    a.id = "a";
    a.name = "A";
    a.version = Version(1, 0, 0);
    a.dependencies = {{"b", "*"}};

    ModManifest b;
    b.id = "b";
    b.name = "B";
    b.version = Version(1, 0, 0);

    const ResolveResult result = resolveMods({a, b}, {"a"});
    REQUIRE(result.ok());
    REQUIRE(result.order.size() == 2);

    const auto indexA = std::find(result.order.begin(), result.order.end(), "a");
    const auto indexB = std::find(result.order.begin(), result.order.end(), "b");
    CHECK(indexB < indexA);
}

TEST_CASE("Dependency cycles are detected") {
    ModManifest c;
    c.id = "c";
    c.name = "C";
    c.version = Version(1, 0, 0);
    c.dependencies = {{"d", "*"}};

    ModManifest d;
    d.id = "d";
    d.name = "D";
    d.version = Version(1, 0, 0);
    d.dependencies = {{"c", "*"}};

    const ResolveResult result = resolveMods({c, d}, {"c"});
    CHECK_FALSE(result.cycles.empty());
}

TEST_CASE("Package helpers know the extension, content types and permissions") {
    CHECK(hasPackageExtension("ana.karts-1.0.0.simmod"));
    CHECK(hasPackageExtension("MOD.SIMMOD"));
    CHECK_FALSE(hasPackageExtension("mod.zip"));
    CHECK(isKnownContentType("car"));
    CHECK(isKnownContentType("collection"));
    CHECK_FALSE(isKnownContentType("cars"));
    CHECK(isKnownPermission("hud"));
    CHECK(isKnownPermission("vehicle.tuning"));
    CHECK_FALSE(isKnownPermission("net"));
}

TEST_CASE("Mod ids are namespaced and content references are mod/content") {
    CHECK(isValidModId("base.core"));
    CHECK(isValidModId("ana.karts"));
    CHECK(isValidModId("liga-2026.round1"));
    CHECK(isValidModId("org.project.mod"));
    CHECK_FALSE(isValidModId("example_mod"));
    CHECK_FALSE(isValidModId("Base.core"));
    CHECK_FALSE(isValidModId("base."));
    CHECK_FALSE(isValidModId(".core"));
    CHECK_FALSE(isValidModId("base.cor e"));

    CHECK(isLegacyModId("example_mod"));
    CHECK_FALSE(isLegacyModId("example.mod"));

    CHECK(isValidContentId("sunset_circuit"));
    CHECK_FALSE(isValidContentId("Sunset"));
    CHECK(isValidContentRef("base.core/voyager"));
    CHECK(isValidContentRef("example_mod/hatchback"));
    CHECK_FALSE(isValidContentRef("voyager"));
    CHECK_FALSE(isValidContentRef("base.core/voyager/extra"));
}

TEST_CASE("A v2 manifest parses every documented field") {
    const std::string text = "format = 2\n"
                             "id = \"ana.karts\"\n"
                             "name = \"Ana's Karts\"\n"
                             "version = \"1.2.0\"\n"
                             "authors = [\"Ana Perez <ana@example.org>\"]\n"
                             "license = \"CC-BY-SA-4.0\"\n"
                             "description = \"A light kart.\"\n"
                             "cover = \"cover.png\"\n"
                             "min_game_version = \"0.1.0\"\n"
                             "types = [\"car\", \"skins\"]\n"
                             "permissions = [\"hud\"]\n"
                             "overrides = [\"base.core/tracks/sunset_circuit/model/track.gltf\"]\n"
                             "conflicts = [{ id = \"other.karts\", version = \"*\", reason = \"kart slot\" }]\n"
                             "dependencies = [{ id = \"base.core\", version = \"^0.1.0\" }]\n"
                             "[urls]\n"
                             "website = \"https://example.org\"\n"
                             "source = \"https://github.com/ana/karts\"\n";

    const ManifestResult result = parseManifest(text, "ana.karts");
    REQUIRE(result.ok());
    CHECK(result.warnings.empty());
    const ModManifest& manifest = *result.manifest;
    CHECK(manifest.format == 2);
    CHECK_FALSE(manifest.migratedFromV1);
    CHECK(manifest.id == "ana.karts");
    CHECK(manifest.name == "Ana's Karts");
    CHECK(manifest.version == Version(1, 2, 0));
    CHECK(manifest.authors == std::vector<std::string>{"Ana Perez <ana@example.org>"});
    CHECK(manifest.license == "CC-BY-SA-4.0");
    CHECK(manifest.description == "A light kart.");
    CHECK(manifest.cover == "cover.png");
    CHECK(manifest.types == std::vector<std::string>{"car", "skins"});
    CHECK(manifest.permissions == std::vector<std::string>{"hud"});
    CHECK(manifest.overrides == std::vector<std::string>{"base.core/tracks/sunset_circuit/model/track.gltf"});
    REQUIRE(manifest.dependencies.size() == 1);
    CHECK(manifest.dependencies[0].id == "base.core");
    CHECK(manifest.dependencies[0].versionRange == "^0.1.0");
    REQUIRE(manifest.conflicts.size() == 1);
    CHECK(manifest.conflicts[0].id == "other.karts");
    CHECK(manifest.conflicts[0].reason == "kart slot");
    CHECK(manifest.website == "https://example.org");
    CHECK(manifest.sourceUrl == "https://github.com/ana/karts");
}

TEST_CASE("A v1 manifest is migrated in memory with warnings") {
    const std::string text = "id = \"old_mod\"\n"
                             "name = \"Old\"\n"
                             "version = \"1.0.0\"\n"
                             "author = \"Someone\"\n"
                             "license = \"CC BY-SA 4.0\"\n"
                             "entry_script = \"scripts/main.lua\"\n"
                             "priority = 5\n"
                             "dependencies = [\"base\", \"tyres >=1.2.0\"]\n";

    const ManifestResult result = parseManifest(text, "old_mod");
    REQUIRE(result.ok());
    CHECK_FALSE(result.warnings.empty());
    const ModManifest& manifest = *result.manifest;
    CHECK(manifest.migratedFromV1);
    CHECK(manifest.format == 2);
    CHECK(manifest.id == "old_mod");
    CHECK(manifest.authors == std::vector<std::string>{"Someone"});
    CHECK(manifest.entryScript == "scripts/main.lua");
    CHECK(manifest.priority == 5);
    REQUIRE(manifest.dependencies.size() == 2);
    CHECK(manifest.dependencies[0].id == "base");
    CHECK(manifest.dependencies[1].id == "tyres");
    CHECK(manifest.dependencies[1].versionRange == ">=1.2.0");
}

TEST_CASE("A v2 manifest rejects unknown types, permissions and overrides") {
    const std::string badType = "format = 2\nid = \"a.b\"\nname = \"B\"\nversion = \"1.0.0\"\n"
                                "authors = [\"A\"]\nlicense = \"MIT\"\ntypes = [\"cars\"]\n";
    CHECK_FALSE(parseManifest(badType, "a.b").ok());

    const std::string badPermission = "format = 2\nid = \"a.b\"\nname = \"B\"\nversion = \"1.0.0\"\n"
                                      "authors = [\"A\"]\nlicense = \"MIT\"\ntypes = [\"apps\"]\n"
                                      "permissions = [\"net\"]\n";
    CHECK_FALSE(parseManifest(badPermission, "a.b").ok());

    const std::string badOverride = "format = 2\nid = \"a.b\"\nname = \"B\"\nversion = \"1.0.0\"\n"
                                    "authors = [\"A\"]\nlicense = \"MIT\"\ntypes = [\"car\"]\n"
                                    "overrides = [\"../escape.gltf\"]\n";
    CHECK_FALSE(parseManifest(badOverride, "a.b").ok());

    const std::string missingTypes = "format = 2\nid = \"a.b\"\nname = \"B\"\nversion = \"1.0.0\"\n"
                                     "authors = [\"A\"]\nlicense = \"MIT\"\n";
    CHECK_FALSE(parseManifest(missingTypes, "a.b").ok());

    const std::string badFormat = "format = 3\nid = \"a.b\"\nname = \"B\"\nversion = \"1.0.0\"\n"
                                  "authors = [\"A\"]\nlicense = \"MIT\"\ntypes = [\"car\"]\n";
    CHECK_FALSE(parseManifest(badFormat, "a.b").ok());
}

namespace {

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

} // namespace

TEST_CASE("developer mode notices a saved file, once, and only when enabled") {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "sim_tests_developermode";
    fs::remove_all(root);
    writeText(root / "content" / "base" / "mod.toml",
              "format = 2\nid = \"base.core\"\nname = \"Base\"\nversion = \"0.1.0\"\n"
              "authors = [\"A\"]\nlicense = \"MIT\"\ntypes = [\"car\"]\n");
    writeText(root / "mods" / "ana.karts" / "mod.toml",
              "format = 2\nid = \"ana.karts\"\nname = \"Karts\"\nversion = \"1.0.0\"\n"
              "authors = [\"A\"]\nlicense = \"MIT\"\ntypes = [\"car\"]\n");

    sim::vfs::VirtualFileSystem vfs;
    sim::core::Config config((root / "config.toml").string());
    ModManager::Paths paths;
    paths.baseContent = root / "content" / "base";
    paths.modsDirectory = root / "mods";

    ModManager manager(vfs, config, paths);
    manager.reload();

    // Disabled by default, and a disabled watcher never reports a change.
    CHECK_FALSE(manager.developer());
    CHECK_FALSE(manager.pollDevelopment());

    manager.setDeveloper(true);
    CHECK(manager.developer());
    // Enabling takes a baseline, so nothing has changed yet.
    CHECK_FALSE(manager.pollDevelopment());

    // Adding any file to a watched source is one change, reported exactly once.
    writeText(root / "mods" / "ana.karts" / "cars" / "kart" / "car.toml", "id = \"kart\"\n");
    CHECK(manager.pollDevelopment());
    CHECK_FALSE(manager.pollDevelopment());

    manager.setDeveloper(false);
    CHECK_FALSE(manager.pollDevelopment());

    fs::remove_all(root);
}
