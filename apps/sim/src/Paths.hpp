// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <filesystem>

namespace sim::core {
class Config;
}

namespace sim::app {

// The directory holding the running executable.
std::filesystem::path executableDirectory();

// The project/user root: the first ancestor of the working directory (or of the
// executable) that holds `content/base`, falling back to the executable
// directory. Shared by the windowed app and the windowless mod commands, so a
// file association that launches `sim` from anywhere still finds the install.
std::filesystem::path locateRootDirectory();

// The platform's per-user data directory for the game (`%APPDATA%/Car Simulator/`
// on Windows, `$XDG_DATA_HOME/Car Simulator/` elsewhere).
std::filesystem::path userDataDirectory();

// Where mods are installed and scanned from: the `mods.directory` config key
// when set, otherwise `<userDataDirectory>/mods`. Never the game folder, which
// may be read-only. Shared by the app and the windowless install command so a
// mod dropped on the window and one installed by the CLI land in the same place.
std::filesystem::path locateModsDirectory(const core::Config& config);

} // namespace sim::app
