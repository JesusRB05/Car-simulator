// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace sim::modding {

// Semantic version (semver 2.0.0). Build metadata is ignored for comparisons.
class Version {
public:
    Version() = default;
    Version(int major, int minor, int patch, std::string prerelease = {});

    static std::optional<Version> parse(std::string_view text);

    int major() const { return major_; }
    int minor() const { return minor_; }
    int patch() const { return patch_; }
    const std::string& prerelease() const { return prerelease_; }

    std::string toString() const;

    bool operator==(const Version& other) const;
    bool operator!=(const Version& other) const { return !(*this == other); }
    bool operator<(const Version& other) const;
    bool operator<=(const Version& other) const { return !(other < *this); }
    bool operator>(const Version& other) const { return other < *this; }
    bool operator>=(const Version& other) const { return !(*this < other); }

private:
    int major_ = 0;
    int minor_ = 0;
    int patch_ = 0;
    std::string prerelease_;
};

// Returns true when `version` satisfies the range expression.
//
// Supported syntax (all comparators ANDed, alternatives separated by "||"):
//   "*"                      any version
//   "1.2.3", "=1.2.3"        exact match
//   "1.2", "1"               prefix match
//   ">=1.2.0", ">1.2.0"      comparison
//   "<=1.2.0", "<1.2.0"      comparison
//   "^1.2.3"                 >=1.2.3 and <2.0.0 (0.x rules respected)
//   "~1.2.3", "~1.2"         >=1.2.3 and <1.3.0
//   ">=1.0.0 <2.0.0"         multiple comparators
bool satisfies(std::string_view range, const Version& version);

} // namespace sim::modding
