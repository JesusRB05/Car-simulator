// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <doctest/doctest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "modding/Fingerprint.hpp"

using namespace sim::modding;

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

ContentSource source(const fs::path& root, const std::string& id, const std::string& version,
                     std::vector<std::string> types) {
    return {id, version, std::move(types), root / id};
}

} // namespace

TEST_CASE("fingerprint: only content that changes the simulation counts") {
    CHECK(affectsSimulation({"car"}));
    CHECK(affectsSimulation({"track"}));
    CHECK(affectsSimulation({"patches"}));
    CHECK(affectsSimulation({"tyres", "skins"}));
    CHECK(affectsSimulation({"plugins"}));

    // The whole point: a livery is not something to force on other players.
    CHECK_FALSE(affectsSimulation({"skins"}));
    CHECK_FALSE(affectsSimulation({"sounds"}));
    CHECK_FALSE(affectsSimulation({"apps"}));
    CHECK_FALSE(affectsSimulation({"locale"}));
    CHECK_FALSE(affectsSimulation({"visuals"}));
    CHECK_FALSE(affectsSimulation({"input"}));
    CHECK_FALSE(affectsSimulation({"collection"}));
    CHECK_FALSE(affectsSimulation({}));
}

TEST_CASE("fingerprint: the digest depends on content and version, not on order") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_fingerprint";
    fs::remove_all(root);

    writeFile(root / "base.core" / "mod.toml", "format = 2\n");
    writeFile(root / "ana.karts" / "cars" / "kart" / "car.toml", "mass = 500\n");
    writeFile(root / "ana.karts" / "skins" / "red" / "livery.toml", "colour = \"red\"\n");
    writeFile(root / "bob.circuit" / "tracks" / "hill" / "track.toml", "length = 1200\n");
    writeFile(root / "carl.hud" / "apps" / "hud" / "main.lua", "print('hi')\n");

    const std::vector<ContentSource> sources = {
        source(root, "base.core", "1.0.0", {"car", "track"}), source(root, "ana.karts", "1.0.0", {"car", "skins"}),
        source(root, "bob.circuit", "2.1.0", {"track"}),
        source(root, "carl.hud", "1.0.0", {"apps"}), // cosmetic: not fingerprinted
    };

    const ContentFingerprint fingerprint = fingerprintOf(sources);
    REQUIRE(fingerprint.entries.size() == 3);
    CHECK(fingerprint.entries[0].id == "ana.karts"); // sorted by id
    CHECK(fingerprint.entries[1].id == "base.core");
    CHECK(fingerprint.entries[2].id == "bob.circuit");
    CHECK(fingerprint.hash.size() == 64);

    // The order mods are handed in must not matter: two machines can only
    // compare one string if it does not.
    std::vector<ContentSource> shuffled = sources;
    std::reverse(shuffled.begin(), shuffled.end());
    CHECK(fingerprintOf(shuffled).hash == fingerprint.hash);

    // Changing a file changes the digest...
    writeFile(root / "ana.karts" / "cars" / "kart" / "car.toml", "mass = 501\n");
    ContentFingerprint changed = fingerprintOf(sources);
    CHECK(changed.hash != fingerprint.hash);
    CHECK(changed.entries[0].hash != fingerprint.entries[0].hash);

    // ...and so does a version bump, even with identical files.
    std::vector<ContentSource> bumped = sources;
    bumped[1].version = "1.0.1";
    CHECK(fingerprintOf(bumped).hash != changed.hash);

    // A cosmetic mod dropping out leaves the fingerprint alone.
    std::vector<ContentSource> withoutHud;
    for (const ContentSource& entry : sources) {
        if (entry.id != "carl.hud") {
            withoutHud.push_back(entry);
        }
    }
    CHECK(fingerprintOf(withoutHud).hash == changed.hash);

    fs::remove_all(root);
}

TEST_CASE("fingerprint: a comparison names what is missing, older or different") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_fingerprint_compare";
    fs::remove_all(root);
    writeFile(root / "ana.karts" / "car.toml", "mass = 500\n");
    writeFile(root / "bob.circuit" / "track.toml", "length = 1200\n");
    writeFile(root / "dave.track" / "track.toml", "length = 900\n");

    const ContentFingerprint local = fingerprintOf({
        source(root, "ana.karts", "1.0.0", {"car"}), source(root, "bob.circuit", "1.9.0", {"track"}),
        source(root, "dave.track", "1.0.0", {"track"}), // not asked for
    });
    REQUIRE(local.entries.size() == 3);

    // The session asks for two of the three local mods, plus one it does not
    // have: everything that can go wrong, in one list.
    std::vector<RequiredContent> required = {local.entries[0], local.entries[1]};
    required.push_back({"pepe.f3", "1.0.0", "", ""}); // missing
    required[0].version = "1.2.0";                    // version mismatch
    required[1].hash = std::string(64, 'f');          // same version, other files

    const ContentComparison comparison = compareContent(required, local);
    CHECK_FALSE(comparison.matches());
    CHECK(comparison.requiredCount == 3);
    REQUIRE(comparison.mismatches.size() == 3);
    CHECK(comparison.mismatches[0].reason == ContentMismatch::Reason::VersionMismatch);
    CHECK(comparison.mismatches[0].message.find("1.2.0") != std::string::npos);
    CHECK(comparison.mismatches[1].reason == ContentMismatch::Reason::HashMismatch);
    CHECK(comparison.mismatches[1].message.find("different content") != std::string::npos);
    CHECK(comparison.mismatches[2].reason == ContentMismatch::Reason::Missing);
    CHECK(comparison.mismatches[2].message.find("pepe.f3") != std::string::npos);
    CHECK(comparison.mismatches[2].localVersion.empty());

    // A local mod nobody asked for is reported, not treated as a failure.
    REQUIRE(comparison.extra.size() == 1);
    CHECK(comparison.extra[0] == "dave.track");
    CHECK(comparison.summary().find("0 of 3") != std::string::npos);
    CHECK(comparison.summary().find("3 need attention") != std::string::npos);

    // An exact match, from the fingerprint the local machine produced.
    const ContentComparison matched = compareContent(local.entries, local);
    CHECK(matched.matches());
    CHECK(matched.mismatches.empty());
    CHECK(matched.extra.empty());
    CHECK(matched.summary().find("content matches") != std::string::npos);

    // An entry without a digest only checks the version: a hand-written list
    // still works.
    const ContentComparison versionOnly = compareContent({{"ana.karts", "1.0.0", "", ""}}, local);
    CHECK(versionOnly.matches());

    fs::remove_all(root);
}

TEST_CASE("fingerprint: a required-content file round-trips") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_fingerprint_file";
    fs::remove_all(root);
    writeFile(root / "ana.karts" / "car.toml", "mass = 500\n");

    ContentFingerprint fingerprint = fingerprintOf({source(root, "ana.karts", "1.2.0", {"car"})});
    fingerprint.entries[0].url = "https://example.com/ana.karts-1.2.0.simmod";

    const std::string text = writeRequiredContent(fingerprint);
    const RequiredContentResult parsed = parseRequiredContent(text, "session.toml");
    for (const std::string& error : parsed.errors) {
        MESSAGE("required-content error: " << error);
    }
    REQUIRE(parsed.ok());
    CHECK(parsed.fingerprint.hash == fingerprint.hash);
    REQUIRE(parsed.fingerprint.entries.size() == 1);
    CHECK(parsed.fingerprint.entries[0].id == "ana.karts");
    CHECK(parsed.fingerprint.entries[0].version == "1.2.0");
    CHECK(parsed.fingerprint.entries[0].hash == fingerprint.entries[0].hash);
    CHECK(parsed.fingerprint.entries[0].url == "https://example.com/ana.karts-1.2.0.simmod");
    CHECK(compareContent(parsed.fingerprint.entries, fingerprint).matches());

    // A list without ids or versions is refused with the reason.
    const RequiredContentResult noMods = parseRequiredContent("hash = \"x\"\n", "session.toml");
    CHECK_FALSE(noMods.ok());
    const RequiredContentResult badEntry =
        parseRequiredContent("[[mod]]\nid = \"ana.karts\"\nversion = \"one\"\n", "s.toml");
    CHECK_FALSE(badEntry.ok());
    CHECK_FALSE(parseRequiredContent("[[mod]\nid =", "s.toml").ok());

    fs::remove_all(root);
}
