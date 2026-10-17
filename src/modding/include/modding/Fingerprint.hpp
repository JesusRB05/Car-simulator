// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace sim::modding {

// One mod as the fingerprint sees it.
struct ContentSource {
    std::string id;
    std::string version;
    std::vector<std::string> types;
    std::filesystem::path directory;
};

// One mod a session needs, with the digest of its files. `url` is optional: a
// server can tell a client where to get what it is missing.
struct RequiredContent {
    std::string id;
    std::string version;
    std::string hash; // payload digest; empty means "the version is enough"
    std::string url;
};

struct ContentFingerprint {
    std::vector<RequiredContent> entries; // sorted by id
    std::string hash;                     // digest of the whole set

    bool empty() const { return entries.empty(); }
};

// True for a content type that changes what is simulated: a car, a circuit,
// tyres, a part, a setup, a patch, a mode or a plugin. Skins, sounds, HUD apps,
// locale files, visuals and input mappings do not, so two players can race with
// different liveries (docs/MODS.md section 8).
bool affectsSimulation(const std::vector<std::string>& types);

// The content fingerprint of a set of mods. It only contains the mods that
// affect the simulation, and it is order-independent and platform-independent:
// the same content always produces the same digest, so two machines can compare
// one string.
ContentFingerprint fingerprintOf(const std::vector<ContentSource>& sources);

// Why a required mod does not match.
struct ContentMismatch {
    enum class Reason {
        Missing,
        VersionMismatch,
        HashMismatch, // same version, different files
    };

    RequiredContent required;
    std::string localVersion; // empty when the mod is absent
    std::string localHash;
    Reason reason = Reason::Missing;
    std::string message; // human-readable, for the manager
};

struct ContentComparison {
    std::vector<ContentMismatch> mismatches;
    // Local simulation-affecting mods the peer did not ask about. Informational:
    // they do not stop the session.
    std::vector<std::string> extra;
    // How many mods the session asked for, so the summary can say "2 of 5".
    std::size_t requiredCount = 0;

    bool matches() const { return mismatches.empty(); }
    // One line for the UI: "3 of 5 mods match; 2 need attention".
    std::string summary() const;
};

// Checks what a session requires against the local content.
ContentComparison compareContent(const std::vector<RequiredContent>& required, const ContentFingerprint& local);

// A required-content list as a small TOML document, so a server can publish
// what it needs and a client can check before joining.
std::string writeRequiredContent(const ContentFingerprint& fingerprint);

struct RequiredContentResult {
    ContentFingerprint fingerprint;
    std::vector<std::string> errors;

    bool ok() const { return errors.empty(); }
};

RequiredContentResult parseRequiredContent(std::string_view tomlText, std::string_view source);

} // namespace sim::modding
