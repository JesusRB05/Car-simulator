// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/Version.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <vector>

namespace sim::modding {
namespace {

bool parseNumber(std::string_view text, int& out) {
    if (text.empty()) {
        return false;
    }
    int value = 0;
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end || value < 0) {
        return false;
    }
    out = value;
    return true;
}

std::vector<std::string> splitPrerelease(std::string_view pre) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (start <= pre.size()) {
        const std::size_t dot = pre.find('.', start);
        const std::size_t end = (dot == std::string_view::npos) ? pre.size() : dot;
        parts.emplace_back(pre.substr(start, end - start));
        if (dot == std::string_view::npos) {
            break;
        }
        start = dot + 1;
    }
    return parts;
}

int comparePrerelease(std::string_view a, std::string_view b) {
    if (a.empty() && b.empty()) {
        return 0;
    }
    if (a.empty()) {
        return 1; // release > prerelease
    }
    if (b.empty()) {
        return -1;
    }

    const auto lhs = splitPrerelease(a);
    const auto rhs = splitPrerelease(b);
    const std::size_t count = std::min(lhs.size(), rhs.size());
    for (std::size_t i = 0; i < count; ++i) {
        int lv = 0;
        int rv = 0;
        const bool lNum = parseNumber(lhs[i], lv);
        const bool rNum = parseNumber(rhs[i], rv);
        if (lNum && rNum) {
            if (lv != rv) {
                return lv < rv ? -1 : 1;
            }
        } else if (lNum != rNum) {
            return lNum ? -1 : 1; // numeric identifiers have lower precedence
        } else if (lhs[i] != rhs[i]) {
            return lhs[i] < rhs[i] ? -1 : 1;
        }
    }
    if (lhs.size() != rhs.size()) {
        return lhs.size() < rhs.size() ? -1 : 1;
    }
    return 0;
}

struct Partial {
    int major = 0;
    int minor = 0;
    int patch = 0;
    bool hasMinor = false;
    bool hasPatch = false;
    std::string prerelease;
    bool valid = false;
};

Partial parsePartial(std::string_view text) {
    Partial result;
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) {
        text.remove_suffix(1);
    }
    // Strip build metadata.
    if (const auto plus = text.find('+'); plus != std::string_view::npos) {
        text = text.substr(0, plus);
    }

    std::string_view core = text;
    if (const auto dash = text.find('-'); dash != std::string_view::npos) {
        core = text.substr(0, dash);
        result.prerelease = std::string(text.substr(dash + 1));
    }

    std::vector<std::string_view> segments;
    std::size_t start = 0;
    while (start <= core.size()) {
        const std::size_t dot = core.find('.', start);
        const std::size_t end = (dot == std::string_view::npos) ? core.size() : dot;
        segments.push_back(core.substr(start, end - start));
        if (dot == std::string_view::npos) {
            break;
        }
        start = dot + 1;
    }
    if (segments.empty() || segments.size() > 3) {
        return result;
    }
    if (!parseNumber(segments[0], result.major)) {
        return result;
    }
    if (segments.size() >= 2) {
        if (!parseNumber(segments[1], result.minor)) {
            return result;
        }
        result.hasMinor = true;
    }
    if (segments.size() == 3) {
        if (!parseNumber(segments[2], result.patch)) {
            return result;
        }
        result.hasPatch = true;
    }
    result.valid = true;
    return result;
}

Version makeVersion(const Partial& p) {
    return Version(p.major, p.minor, p.patch, p.prerelease);
}

bool checkComparator(std::string_view token, const Version& version) {
    std::string_view op;
    if (token.size() >= 2 && (token.substr(0, 2) == ">=" || token.substr(0, 2) == "<=")) {
        op = token.substr(0, 2);
        token.remove_prefix(2);
    } else if (!token.empty() && (token.front() == '>' || token.front() == '<' || token.front() == '=' ||
                                  token.front() == '^' || token.front() == '~')) {
        op = token.substr(0, 1);
        token.remove_prefix(1);
    }

    const Partial p = parsePartial(token);
    if (!p.valid) {
        return false;
    }

    if (op == ">=") {
        return version >= makeVersion(p);
    }
    if (op == "<=") {
        return version <= makeVersion(p);
    }
    if (op == ">") {
        return version > makeVersion(p);
    }
    if (op == "<") {
        return version < makeVersion(p);
    }
    if (op == "^") {
        if (version < makeVersion(p)) {
            return false;
        }
        Version upper;
        if (p.major > 0) {
            upper = Version(p.major + 1, 0, 0);
        } else if (p.hasMinor && p.minor > 0) {
            upper = Version(0, p.minor + 1, 0);
        } else {
            upper = Version(0, 0, p.patch + 1);
        }
        return version < upper;
    }
    if (op == "~") {
        if (version < makeVersion(p)) {
            return false;
        }
        const Version upper = p.hasMinor ? Version(p.major, p.minor + 1, 0) : Version(p.major + 1, 0, 0);
        return version < upper;
    }

    // Exact or prefix ("=", or bare).
    if (p.hasMinor && p.hasPatch) {
        return version == makeVersion(p);
    }
    if (version.major() != p.major) {
        return false;
    }
    if (p.hasMinor && version.minor() != p.minor) {
        return false;
    }
    return true;
}

} // namespace

Version::Version(int major, int minor, int patch, std::string prerelease)
    : major_(major), minor_(minor), patch_(patch), prerelease_(std::move(prerelease)) {}

std::optional<Version> Version::parse(std::string_view text) {
    const Partial p = parsePartial(text);
    if (!p.valid || !p.hasMinor || !p.hasPatch) {
        return std::nullopt;
    }
    return makeVersion(p);
}

std::string Version::toString() const {
    std::string result = std::to_string(major_) + "." + std::to_string(minor_) + "." + std::to_string(patch_);
    if (!prerelease_.empty()) {
        result += "-" + prerelease_;
    }
    return result;
}

bool Version::operator==(const Version& other) const {
    return major_ == other.major_ && minor_ == other.minor_ && patch_ == other.patch_ &&
           prerelease_ == other.prerelease_;
}

bool Version::operator<(const Version& other) const {
    if (major_ != other.major_) {
        return major_ < other.major_;
    }
    if (minor_ != other.minor_) {
        return minor_ < other.minor_;
    }
    if (patch_ != other.patch_) {
        return patch_ < other.patch_;
    }
    return comparePrerelease(prerelease_, other.prerelease_) < 0;
}

bool satisfies(std::string_view range, const Version& version) {
    // Trim.
    while (!range.empty() && std::isspace(static_cast<unsigned char>(range.front())) != 0) {
        range.remove_prefix(1);
    }
    while (!range.empty() && std::isspace(static_cast<unsigned char>(range.back())) != 0) {
        range.remove_suffix(1);
    }

    if (range.empty() || range == "*") {
        return true;
    }

    // Split alternatives on "||".
    std::size_t altStart = 0;
    while (altStart <= range.size()) {
        const std::size_t orPos = range.find("||", altStart);
        const std::size_t altEnd = (orPos == std::string_view::npos) ? range.size() : orPos;
        const std::string_view alternative = range.substr(altStart, altEnd - altStart);

        bool allMatch = true;
        bool anyComparator = false;
        std::size_t tokenStart = 0;
        while (tokenStart < alternative.size()) {
            while (tokenStart < alternative.size() &&
                   std::isspace(static_cast<unsigned char>(alternative[tokenStart])) != 0) {
                ++tokenStart;
            }
            std::size_t tokenEnd = tokenStart;
            while (tokenEnd < alternative.size() &&
                   std::isspace(static_cast<unsigned char>(alternative[tokenEnd])) == 0) {
                ++tokenEnd;
            }
            if (tokenEnd > tokenStart) {
                anyComparator = true;
                if (!checkComparator(alternative.substr(tokenStart, tokenEnd - tokenStart), version)) {
                    allMatch = false;
                    break;
                }
            }
            tokenStart = tokenEnd;
        }

        if (allMatch && anyComparator) {
            return true;
        }
        if (orPos == std::string_view::npos) {
            break;
        }
        altStart = orPos + 2;
    }

    return false;
}

} // namespace sim::modding
