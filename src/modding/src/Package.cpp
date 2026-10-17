// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/Package.hpp"

#include <algorithm>
#include <cctype>

namespace sim::modding {

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

} // namespace sim::modding
