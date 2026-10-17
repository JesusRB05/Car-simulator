// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "modding/Collection.hpp"
#include "modding/Profile.hpp"

using namespace sim::modding;

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

} // namespace

TEST_CASE("collection: a pack parses its mod list, ranges and options") {
    const std::string text = R"(
name = "2026 Championship"
description = "Four circuits and two cars"

[[mod]]
id = "ana.karts"

[[mod]]
id = "bob.circuits"
version = "^2.0.0"
reason = "the season's circuits"

[[mod]]
id = "carl.skins"
version = "1.2.3"
optional = true
)";

    const CollectionResult result = parseCollection(text, "collection.toml");
    for (const std::string& error : result.errors) {
        MESSAGE("collection error: " << error);
    }
    REQUIRE(result.ok());
    CHECK(result.collection->name == "2026 Championship");
    REQUIRE(result.collection->entries.size() == 3);
    CHECK(result.collection->entries[0].modId == "ana.karts");
    CHECK(result.collection->entries[0].versionRange == "*");
    CHECK_FALSE(result.collection->entries[0].optional);
    CHECK(result.collection->entries[1].versionRange == "^2.0.0");
    CHECK(result.collection->entries[1].reason == "the season's circuits");
    CHECK(result.collection->entries[2].optional);
}

TEST_CASE("collection: a malformed pack is rejected with the reason") {
    const CollectionResult noId = parseCollection("[[mod]]\nversion = \"1.0.0\"\n", "pack.toml");
    CHECK_FALSE(noId.ok());
    CHECK_FALSE(noId.errors.empty());

    const CollectionResult badId = parseCollection("[[mod]]\nid = \"Not Namespaced\"\n", "pack.toml");
    CHECK_FALSE(badId.ok());

    const CollectionResult broken = parseCollection("[[mod]\nid =", "pack.toml");
    CHECK_FALSE(broken.ok());

    // An empty pack is suspicious but not fatal.
    const CollectionResult empty = parseCollection("name = \"Empty\"\n", "pack.toml");
    CHECK(empty.ok());
    CHECK_FALSE(empty.warnings.empty());
}

TEST_CASE("collection: a plan reports missing and mismatched mods in words") {
    Collection collection;
    collection.id = "ana.championship";
    collection.name = "Championship";
    collection.entries = {
        {"base.core", "^1.0.0", false, ""},
        {"ana.karts", "^1.0.0", false, ""},
        {"bob.circuits", "^2.0.0", false, "the circuits"},
        {"carl.skins", "1.0.0", true, "extra liveries"},
    };

    const std::vector<InstalledMod> installed = {
        {"base.core", "1.2.0"},    // satisfies
        {"ana.karts", "1.0.0"},    // satisfies
        {"bob.circuits", "1.9.0"}, // too old
    };

    const CollectionPlan plan = planCollection(collection, installed);
    REQUIRE(plan.requirements.size() == 4);
    CHECK(plan.requirements[0].state == CollectionRequirement::State::Satisfied);
    CHECK(plan.requirements[1].satisfied());
    CHECK(plan.requirements[2].state == CollectionRequirement::State::VersionMismatch);
    CHECK(plan.requirements[2].installedVersion == "1.9.0");
    CHECK(plan.requirements[3].state == CollectionRequirement::State::Missing);
    CHECK(plan.requirements[3].entry.optional);

    // The version mismatch blocks the pack; the optional miss does not.
    CHECK_FALSE(plan.complete());
    Collection optionalMismatch = collection;
    optionalMismatch.entries[2].optional = true;
    CHECK(planCollection(optionalMismatch, installed).complete());

    const std::vector<std::string> messages = plan.messages();
    REQUIRE(messages.size() == 2);
    CHECK(messages[0].find("bob.circuits") != std::string::npos);
    CHECK(messages[0].find("2.0.0") != std::string::npos);
    CHECK(messages[0].find("1.9.0") != std::string::npos);
    CHECK(messages[1].find("carl.skins") != std::string::npos);
    CHECK(messages[1].find("(optional)") != std::string::npos);

    // Only a genuinely missing (not mismatched) mod is something to download.
    REQUIRE(plan.missingIds().size() == 1);
    CHECK(plan.missingIds()[0] == "carl.skins");
}

TEST_CASE("profile: a store round-trips names, sets and the last applied") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_profile_store";
    fs::remove_all(root);

    ProfileStore store(root / "profiles.toml");
    REQUIRE(store.load()); // missing file: empty, not an error
    CHECK(store.names().empty());
    CHECK_FALSE(store.find("racing").has_value());

    CHECK(store.set({"racing", {"base.core", "ana.karts"}}));
    CHECK(store.set({"vanilla", {}}));
    CHECK_FALSE(store.set({"", {"base.core"}})); // a profile needs a name
    store.setLastApplied("racing");
    REQUIRE(store.save());

    ProfileStore reloaded(root / "profiles.toml");
    REQUIRE(reloaded.load());
    const std::vector<std::string> names = reloaded.names();
    REQUIRE(names.size() == 2);
    CHECK(names[0] == "racing"); // sorted
    CHECK(names[1] == "vanilla");
    const std::optional<Profile> racing = reloaded.find("racing");
    REQUIRE(racing.has_value());
    REQUIRE(racing->mods.size() == 2);
    CHECK(racing->mods[1] == "ana.karts");
    CHECK(reloaded.find("vanilla")->mods.empty());
    CHECK(reloaded.lastApplied() == "racing");

    CHECK(reloaded.remove("vanilla"));
    CHECK_FALSE(reloaded.remove("vanilla"));
    CHECK(reloaded.names().size() == 1);

    fs::remove_all(root);
}

TEST_CASE("profile: one profile exports and imports on its own") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_profile_export";
    fs::remove_all(root);

    ProfileStore store(root / "profiles.toml");
    CHECK(store.set({"racing", {"base.core", "ana.karts", "bob.circuits"}}));
    CHECK(store.set({"drift", {"base.core", "ana.karts"}}));
    REQUIRE(store.save());

    const fs::path shared = root / "shared" / "racing.toml";
    REQUIRE(store.exportProfile("racing", shared));
    CHECK(fs::exists(shared));
    CHECK_FALSE(store.exportProfile("missing", root / "nope.toml"));

    // A second machine imports the file into its own store.
    ProfileStore other(root / "other" / "profiles.toml");
    REQUIRE(other.load());
    REQUIRE(other.importProfile(shared));
    const std::optional<Profile> imported = other.find("racing");
    REQUIRE(imported.has_value());
    REQUIRE(imported->mods.size() == 3);
    CHECK(imported->mods[2] == "bob.circuits");
    // The profile it already had is untouched.
    CHECK_FALSE(other.find("drift").has_value());
    REQUIRE(other.save());

    // An empty or unreadable file is refused, not silently accepted.
    std::string error;
    CHECK_FALSE(other.importProfile(root / "missing.toml", &error));
    CHECK_FALSE(error.empty());

    fs::remove_all(root);
}
