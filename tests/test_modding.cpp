// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <algorithm>

#include "modding/DependencyResolver.hpp"
#include "modding/Manifest.hpp"
#include "modding/Version.hpp"

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
