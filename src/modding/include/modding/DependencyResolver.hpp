// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

#include "modding/Manifest.hpp"

namespace sim::modding {

struct ResolveError
{
    std::string modId;
    std::string message;
};

struct ResolveResult
{
    // Active mods in load order: a mod always appears after its dependencies.
    std::vector<std::string> order;
    // Requested mods that could not be activated (missing or invalid).
    std::vector<std::string> disabled;
    std::vector<ResolveError> errors;
    std::vector<std::string> conflicts; // e.g. two mods sharing an id
    // Detected dependency cycles, each as the list of ids forming the cycle.
    std::vector<std::vector<std::string>> cycles;

    bool ok() const { return errors.empty() && conflicts.empty(); }
};

// Resolves the load order for a set of mods.
//
// * If `requestedIds` is empty, every known mod is requested.
// * Dependencies of requested mods are enabled automatically.
// * Missing dependencies, unsatisfied version ranges, cycles and duplicate ids
//   are reported in `errors` / `conflicts` and disable the affected mods.
// * The result is deterministic: order is a stable topological order.
ResolveResult resolveMods(const std::vector<ModManifest>& manifests, const std::vector<std::string>& requestedIds);

} // namespace sim::modding
