// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <filesystem>
#include <string>

namespace sim::modding {

// Writes the contents of `directory` into a `.simmod` package: a zip whose
// entries are all stored (not compressed), added in sorted order with a fixed
// timestamp. The same content therefore always produces the same bytes, so a
// release can be rebuilt and compared, and packing needs no external tool
// (docs/MODS.md section 9). Returns false and fills `error` on failure.
bool writePackage(const std::filesystem::path& directory, const std::filesystem::path& destination, std::string& error);

} // namespace sim::modding
