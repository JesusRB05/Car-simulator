// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/Fingerprint.hpp"

#include <algorithm>
#include <sstream>

#include <toml++/toml.hpp>

#include "core/Sha256.hpp"
#include "modding/Registry.hpp" // payloadDigest()
#include "modding/Version.hpp"

namespace sim::modding {
namespace {

// The content types that a peer has to have as well. Everything unnamed here
// (liveries, sounds, HUD apps, locale, input, collections) is optional.
constexpr std::string_view kSimulationTypes[] = {"car",    "track",   "parts", "tyres",
                                                 "setups", "patches", "modes", "plugins"};

std::string digestOf(const std::vector<RequiredContent>& entries) {
    core::Sha256 hash;
    for (const RequiredContent& entry : entries) {
        std::string line = entry.id + "\n" + entry.version + "\n" + entry.hash + "\n";
        hash.update(line.data(), line.size());
    }
    return hash.hexDigest();
}

std::string stringField(const toml::table& table, std::string_view key) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<std::string>()) {
            return *value;
        }
    }
    return {};
}

} // namespace

bool affectsSimulation(const std::vector<std::string>& types) {
    for (const std::string& type : types) {
        if (std::find(std::begin(kSimulationTypes), std::end(kSimulationTypes), type) != std::end(kSimulationTypes)) {
            return true;
        }
    }
    return false;
}

ContentFingerprint fingerprintOf(const std::vector<ContentSource>& sources) {
    ContentFingerprint fingerprint;
    for (const ContentSource& source : sources) {
        if (source.id.empty() || !affectsSimulation(source.types)) {
            continue;
        }
        RequiredContent entry;
        entry.id = source.id;
        entry.version = source.version;
        entry.hash = payloadDigest(source.directory);
        fingerprint.entries.push_back(std::move(entry));
    }
    // Sorted by id, so the digest does not depend on the order mods are loaded.
    std::sort(fingerprint.entries.begin(), fingerprint.entries.end(),
              [](const RequiredContent& a, const RequiredContent& b) { return a.id < b.id; });
    fingerprint.hash = digestOf(fingerprint.entries);
    return fingerprint;
}

std::string ContentComparison::summary() const {
    std::ostringstream oss;
    if (matches()) {
        oss << "content matches (" << requiredCount << " mod(s) checked";
        if (!extra.empty()) {
            oss << ", plus " << extra.size() << " extra local mod(s)";
        }
        oss << ")";
        return oss.str();
    }
    oss << (requiredCount - mismatches.size()) << " of " << requiredCount << " mod(s) match; " << mismatches.size()
        << (mismatches.size() == 1 ? " needs" : " need") << " attention";
    return oss.str();
}

ContentComparison compareContent(const std::vector<RequiredContent>& required, const ContentFingerprint& local) {
    ContentComparison comparison;
    comparison.requiredCount = required.size();

    for (const RequiredContent& entry : required) {
        const auto found =
            std::find_if(local.entries.begin(), local.entries.end(),
                         [&entry](const RequiredContent& candidate) { return candidate.id == entry.id; });

        ContentMismatch mismatch;
        mismatch.required = entry;
        bool mismatched = true;
        if (found == local.entries.end()) {
            mismatch.reason = ContentMismatch::Reason::Missing;
            mismatch.message = "the session needs '" + entry.id + "' " + entry.version + ", which is not installed";
        } else {
            mismatch.localVersion = found->version;
            mismatch.localHash = found->hash;
            // A version mismatch is reported whatever the files look like; a
            // hash mismatch means the same version with different content,
            // which is exactly the case that would desync silently.
            if (!entry.version.empty() && found->version != entry.version) {
                mismatch.reason = ContentMismatch::Reason::VersionMismatch;
                mismatch.message = "the session needs '" + entry.id + "' " + entry.version + " but " + found->version +
                                   " is installed";
            } else if (!entry.hash.empty() && found->hash != entry.hash) {
                mismatch.reason = ContentMismatch::Reason::HashMismatch;
                mismatch.message =
                    "'" + entry.id + "' has different content than the session (same version, different files)";
            } else {
                mismatched = false;
            }
        }
        if (mismatched) {
            comparison.mismatches.push_back(std::move(mismatch));
        }
    }

    for (const RequiredContent& entry : local.entries) {
        const bool asked = std::any_of(required.begin(), required.end(),
                                       [&entry](const RequiredContent& candidate) { return candidate.id == entry.id; });
        if (!asked) {
            comparison.extra.push_back(entry.id);
        }
    }
    return comparison;
}

std::string writeRequiredContent(const ContentFingerprint& fingerprint) {
    std::ostringstream oss;
    oss << "# Required content for a session (docs/MODS.md section 8).\n";
    oss << "hash = \"" << fingerprint.hash << "\"\n";
    for (const RequiredContent& entry : fingerprint.entries) {
        oss << "\n[[mod]]\n";
        oss << "id = \"" << entry.id << "\"\n";
        oss << "version = \"" << entry.version << "\"\n";
        oss << "hash = \"" << entry.hash << "\"\n";
        if (!entry.url.empty()) {
            oss << "url = \"" << entry.url << "\"\n";
        }
    }
    return oss.str();
}

RequiredContentResult parseRequiredContent(std::string_view tomlText, std::string_view source) {
    RequiredContentResult result;
    const std::string where(source);

    toml::table table;
    try {
        table = toml::parse(tomlText, source);
    } catch (const toml::parse_error& error) {
        result.errors.push_back(where + ": " + std::string(error.description()));
        return result;
    }

    result.fingerprint.hash = stringField(table, "hash");

    const toml::array* mods = table.get_as<toml::array>("mod");
    if (mods == nullptr) {
        result.errors.push_back(where + ": the list has no [[mod]] entries");
        return result;
    }

    int index = 0;
    for (const toml::node& node : *mods) {
        ++index;
        const toml::table* entry = node.as_table();
        if (entry == nullptr) {
            result.errors.push_back(where + ": [[mod]] #" + std::to_string(index) + " is not a table");
            continue;
        }
        RequiredContent parsed;
        parsed.id = stringField(*entry, "id");
        parsed.version = stringField(*entry, "version");
        parsed.hash = stringField(*entry, "hash");
        parsed.url = stringField(*entry, "url");

        const std::string prefix = where + ": [[mod]] #" + std::to_string(index) + " ";
        if (parsed.id.empty() || !Version::parse(parsed.version)) {
            result.errors.push_back(prefix + "needs an id and a version ('" + parsed.id + "' '" + parsed.version +
                                    "')");
            continue;
        }
        result.fingerprint.entries.push_back(std::move(parsed));
    }

    std::sort(result.fingerprint.entries.begin(), result.fingerprint.entries.end(),
              [](const RequiredContent& a, const RequiredContent& b) { return a.id < b.id; });
    return result;
}

} // namespace sim::modding
