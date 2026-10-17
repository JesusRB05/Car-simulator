// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/Package.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>

namespace sim::modding {
namespace {

// DOS device names are still reserved on Windows, with or without an extension.
bool isReservedName(std::string_view stem) {
    static constexpr std::string_view kReserved[] = {"con",  "prn",  "aux",  "nul",  "com1", "com2", "com3", "com4",
                                                     "com5", "com6", "com7", "com8", "com9", "lpt1", "lpt2", "lpt3",
                                                     "lpt4", "lpt5", "lpt6", "lpt7", "lpt8", "lpt9"};
    if (stem.empty() || stem.size() > 4) {
        return false;
    }
    std::string lower(stem);
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return std::any_of(std::begin(kReserved), std::end(kReserved),
                       [&lower](std::string_view known) { return known == lower; });
}

} // namespace

bool isKnownContentType(std::string_view type) {
    return std::any_of(std::begin(kContentTypes), std::end(kContentTypes),
                       [type](std::string_view known) { return known == type; });
}

bool isKnownPermission(std::string_view permission) {
    return std::any_of(std::begin(kScriptPermissions), std::end(kScriptPermissions),
                       [permission](std::string_view known) { return known == permission; });
}

bool hasPackageExtension(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return extension == kPackageExtension;
}

bool isSafePackagePath(std::string_view path) {
    if (path.empty() || path.find('\0') != std::string_view::npos) {
        return false;
    }
    // A backslash is a directory separator on Windows and a legal character
    // nowhere useful inside a package, so it is rejected outright.
    if (path.find('\\') != std::string_view::npos) {
        return false;
    }
    if (path.front() == '/') {
        return false; // absolute
    }
    if (path.size() >= 2 && path[1] == ':') {
        return false; // drive letter
    }

    std::size_t start = 0;
    while (start <= path.size()) {
        const std::size_t slash = path.find('/', start);
        const std::string_view segment =
            slash == std::string_view::npos ? path.substr(start) : path.substr(start, slash - start);
        if (segment == "." || segment == "..") {
            return false;
        }
        // Characters that are illegal on Windows (and so would make the file
        // impossible to write there) plus control characters.
        for (const char ch : segment) {
            if (ch < 0x20 || ch == '<' || ch == '>' || ch == ':' || ch == '"' || ch == '|' || ch == '?' || ch == '*') {
                return false;
            }
        }
        // Reserved DOS device names, with or without an extension.
        const std::size_t dot = segment.find('.');
        const std::string_view stem = dot == std::string_view::npos ? segment : segment.substr(0, dot);
        if (isReservedName(stem)) {
            return false;
        }
        if (slash == std::string_view::npos) {
            break;
        }
        start = slash + 1;
    }
    return true;
}

bool isNativeBinaryPath(std::string_view path) {
    const std::filesystem::path file(path);
    std::string extension = file.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    static constexpr std::string_view kBinaries[] = {".exe", ".dll", ".so", ".dylib", ".bat", ".cmd", ".ps1"};
    const bool binary = std::any_of(std::begin(kBinaries), std::end(kBinaries),
                                    [&extension](std::string_view known) { return known == extension; });
    if (!binary) {
        return false;
    }
    // Native code is only ever expected in `plugins/`, where the loader asks the
    // user before loading anything.
    return !path.starts_with("plugins/");
}

} // namespace sim::modding
