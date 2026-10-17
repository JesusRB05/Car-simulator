// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "core/Sha256.hpp"
#include "modding/Catalog.hpp"
#include "net/HttpClient.hpp"

using namespace sim::modding;
namespace net = sim::net;
namespace core = sim::core;

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

// A 64-character digest that is at least shaped like a real one.
std::string fakeDigest(char fill) {
    return std::string(64, fill);
}

std::string indexEntry(const std::string& id, const std::string& version, const std::string& download,
                       const std::string& sha = fakeDigest('a'), std::uint64_t size = 100) {
    return "\n[[version]]\nid = \"" + id + "\"\nversion = \"" + version + "\"\ndownload = \"" + download +
           "\"\nsha256 = \"" + sha + "\"\nsize = " + std::to_string(size) + "\n";
}

} // namespace

TEST_CASE("catalog: an index parses, and bad entries are named") {
    const std::string text = std::string("name = \"Team index\"\n") +
                             indexEntry("ana.karts", "1.2.0", "file:///m/a.simmod") +
                             indexEntry("bob.circuits", "2.0.0", "file:///m/b.simmod");

    const Catalog catalog = parseCatalog(text, "index.toml");
    for (const std::string& error : catalog.errors) {
        MESSAGE("catalog error: " << error);
    }
    REQUIRE(catalog.ok());
    CHECK(catalog.name == "Team index");
    REQUIRE(catalog.versions.size() == 2);
    CHECK(catalog.versions[0].id == "ana.karts");
    CHECK(catalog.versions[0].size == 100);
    CHECK(catalog.ids() == std::vector<std::string>{"ana.karts", "bob.circuits"});

    // A digest of the wrong shape is refused: a half-checksum is worse than none.
    const Catalog badDigest = parseCatalog(indexEntry("ana.karts", "1.0.0", "file:///a", "abc"), "index.toml");
    CHECK_FALSE(badDigest.ok());
    CHECK(badDigest.errors[0].find("SHA-256") != std::string::npos);

    // A non-namespaced id, a broken version and a missing URL are all refused.
    CHECK_FALSE(parseCatalog(indexEntry("Not Namespaced", "1.0.0", "file:///a"), "index.toml").ok());
    CHECK_FALSE(parseCatalog(indexEntry("ana.karts", "one.two", "file:///a"), "index.toml").ok());
    CHECK_FALSE(parseCatalog(indexEntry("ana.karts", "1.0.0", ""), "index.toml").ok());

    // Two identical versions are a mistake in the index, not a coin flip.
    const Catalog duplicate = parseCatalog(
        indexEntry("ana.karts", "1.0.0", "file:///a") + indexEntry("ana.karts", "1.0.0", "file:///b"), "index.toml");
    CHECK_FALSE(duplicate.ok());

    // A syntactically broken index never crashes.
    CHECK_FALSE(parseCatalog("[[version]\nid =", "index.toml").ok());
    CHECK_FALSE(parseCatalog("name = \"no versions\"\n", "index.toml").ok());
}

TEST_CASE("catalog: the latest version prefers a stable release") {
    const std::string text = indexEntry("ana.karts", "1.0.0", "file:///1") +
                             indexEntry("ana.karts", "1.4.0", "file:///14") +
                             indexEntry("ana.karts", "2.0.0-beta.1", "file:///2b") +
                             indexEntry("ana.karts", "2.0.0", "file:///2") + indexEntry("bob.x", "1.0.0", "file:///x");
    const Catalog catalog = parseCatalog(text, "index.toml");
    REQUIRE(catalog.ok());

    const CatalogVersion* latest = catalog.latest("ana.karts");
    REQUIRE(latest != nullptr);
    CHECK(latest->version == "2.0.0");
    CHECK(latest->downloadUrl == "file:///2");

    // A prerelease is returned only when nothing stable exists.
    const Catalog betaOnly = parseCatalog(indexEntry("ana.karts", "1.0.0", "file:///1") +
                                              indexEntry("ana.karts", "1.1.0-rc.1", "file:///rc"),
                                          "index.toml");
    REQUIRE(betaOnly.latest("ana.karts") != nullptr);
    CHECK(betaOnly.latest("ana.karts")->version == "1.1.0-rc.1");
    CHECK(betaOnly.latest("nobody.here") == nullptr);

    // Search filters by substring and by type.
    const Catalog typed = parseCatalog(indexEntry("ana.karts", "1.0.0", "file:///a") + "types = [\"car\"]\n" +
                                           indexEntry("bob.circuits", "1.0.0", "file:///b") + "types = [\"track\"]\n",
                                       "index.toml");
    REQUIRE(typed.ok());
    CHECK(typed.search("kart", "").size() == 1);
    CHECK(typed.search("", "track").size() == 1);
    CHECK(typed.search("", "car")[0].id == "ana.karts");
    CHECK(typed.search("", "").size() == 2);
}

TEST_CASE("catalog: updates are newer versions, and prereleases are not offered") {
    const std::string text =
        indexEntry("ana.karts", "1.0.0", "file:///old") + indexEntry("ana.karts", "1.2.0", "file:///new") +
        indexEntry("ana.karts", "1.3.0-rc.1", "file:///rc") + indexEntry("bob.circuits", "2.0.0", "file:///b") +
        indexEntry("carl.news", "1.0.0", "file:///n");
    const Catalog catalog = parseCatalog(text, "index.toml");
    REQUIRE(catalog.ok());

    const std::vector<InstalledMod> installed = {
        {"ana.karts", "1.0.0"},    // has an update
        {"bob.circuits", "2.0.0"}, // already current
        {"dave.unknown", "1.0.0"}, // not in the index
    };
    const std::vector<CatalogUpdate> updates = catalogUpdates(catalog, installed);
    REQUIRE(updates.size() == 1);
    CHECK(updates[0].available.id == "ana.karts");
    CHECK(updates[0].available.version == "1.2.0"); // not the rc
    CHECK(updates[0].installedVersion == "1.0.0");

    // Nothing in the index is newer than what is installed: no updates.
    CHECK(catalogUpdates(catalog, {{"ana.karts", "1.3.0"}}).empty());
    CHECK(catalogUpdates(catalog, {{"ana.karts", "9.9.9"}}).empty());
}

TEST_CASE("catalog: a download is verified before it is written") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_catalog";
    fs::remove_all(root);

    const std::string payload = "not really a zip, but bytes are bytes\n";
    writeFile(root / "remote" / "ana.karts-1.2.0.simmod", payload);

    const std::unique_ptr<net::HttpClient> http = net::makeHttpClient();
    CatalogVersion entry;
    entry.id = "ana.karts";
    entry.version = "1.2.0";
    entry.downloadUrl = "file:///" + (root / "remote" / "ana.karts-1.2.0.simmod").generic_string();
    entry.sha256 = core::sha256Hex(payload.data(), payload.size());
    entry.size = payload.size();

    const fs::path destination = root / "cache" / "ana.karts-1.2.0.simmod";
    const DownloadResult good = downloadEntry(*http, entry, destination);
    for (const std::string& error : {good.error}) {
        MESSAGE("download error: " << error);
    }
    REQUIRE(good.ok);
    CHECK(readFile(destination) == payload);

    // A digest that does not match discards the file instead of leaving it there.
    CatalogVersion tampered = entry;
    tampered.sha256 = fakeDigest('b');
    const fs::path second = root / "cache" / "tampered.simmod";
    const DownloadResult rejected = downloadEntry(*http, tampered, second);
    CHECK_FALSE(rejected.ok);
    CHECK(rejected.error.find("SHA-256") != std::string::npos);
    CHECK_FALSE(fs::exists(second));

    // A size that does not match is caught even before the digest.
    CatalogVersion wrongSize = entry;
    wrongSize.size = payload.size() + 1;
    CHECK_FALSE(downloadEntry(*http, wrongSize, root / "cache" / "size.simmod").ok);

    // A missing file is an error, not an empty install.
    CatalogVersion missing = entry;
    missing.downloadUrl = "file:///" + (root / "remote" / "nope.simmod").generic_string();
    const DownloadResult notFound = downloadEntry(*http, missing, root / "cache" / "missing.simmod");
    CHECK_FALSE(notFound.ok);
    CHECK_FALSE(notFound.error.empty());

    // https needs a transport this build does not have, and says so.
    CatalogVersion secure = entry;
    secure.downloadUrl = "https://example.com/ana.karts-1.2.0.simmod";
    const DownloadResult noTls = downloadEntry(*http, secure, root / "cache" / "secure.simmod");
    CHECK_FALSE(noTls.ok);
    CHECK(noTls.error.find("transport") != std::string::npos);
    CHECK_FALSE(fs::exists(root / "cache" / "secure.simmod"));

    fs::remove_all(root);
}

TEST_CASE("catalog: a local index loads through the same path an https one would") {
    const fs::path root = fs::temp_directory_path() / "sim_tests_catalog_index";
    fs::remove_all(root);
    writeFile(root / "index.toml", std::string("name = \"Local\"\n") + indexEntry("ana.karts", "1.0.0", "file:///a"));

    const std::unique_ptr<net::HttpClient> http = net::makeHttpClient();

    // A plain path and a file:// URL are the same thing.
    const Catalog byPath = loadCatalog(*http, (root / "index.toml").string());
    REQUIRE(byPath.ok());
    CHECK(byPath.versions.size() == 1);
    const Catalog byUrl = loadCatalog(*http, "file:///" + (root / "index.toml").generic_string());
    REQUIRE(byUrl.ok());

    // A missing index explains itself.
    const Catalog missing = loadCatalog(*http, (root / "nope.toml").string());
    CHECK_FALSE(missing.ok());
    CHECK_FALSE(missing.errors.empty());

    // `resolveLocalUrl` is what decides whether a URL is a path at all.
    std::string path;
    CHECK(net::resolveLocalUrl("file:///C:/mods/index.toml", path));
    CHECK(path == "C:/mods/index.toml");
    CHECK(net::resolveLocalUrl("mods/index.toml", path));
    CHECK(path == "mods/index.toml");
    CHECK_FALSE(net::resolveLocalUrl("https://example.com/index.toml", path));
    CHECK_FALSE(net::resolveLocalUrl("", path));

    fs::remove_all(root);
}
