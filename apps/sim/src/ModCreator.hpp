// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

namespace sim::app {

// `sim mod new <type> <mod-id>/<name> [--from <ref>] [--for <car-ref>] [--out <dir>]`
struct ModNewOptions {
    std::string type;   // car | track | skin | patch
    std::string target; // "<mod-id>/<name>"
    std::string from;   // optional content reference to start from
    std::string forCar; // for a skin: the car it dresses, "<mod>/<car>"
    std::string output; // where to scaffold (default: the user mods directory)
};

// A content reference split into its parts.
struct ContentRef {
    std::string modId;
    std::string id;
};

// "<mod-id>/<content-id>" -> its two halves. The id is accepted without a
// namespace too, for the base content and legacy mods.
ContentRef splitRef(const std::string& ref);

// Scaffolds a mod. Returns a process exit code.
int createMod(const ModNewOptions& options);

// Validates a mod folder or `.simmod`: the manifest, the folder layout and
// every descriptor (cars, tracks, patches, skins, parts), using the same
// parsers the game loads with. Returns a process exit code.
int validateMod(const std::string& path);

// Packs a mod folder into a `.simmod`, then prints its digest and the index
// entry for it. Returns a process exit code.
int packMod(const std::string& path, const std::string& output);

} // namespace sim::app
