// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/DependencyResolver.hpp"

#include <algorithm>
#include <unordered_map>

namespace sim::modding {
namespace {

enum class VisitState { Unvisited, Visiting, Done };

struct Resolver {
    const std::unordered_map<std::string, const ModManifest*>& index;
    std::unordered_map<std::string, VisitState> state;
    std::unordered_map<std::string, bool> valid;
    std::vector<std::string> stack;
    std::vector<std::string> order;
    ResolveResult* result = nullptr;

    bool visit(const std::string& id) {
        const auto stateIt = state.find(id);
        if (stateIt != state.end() && stateIt->second == VisitState::Visiting) {
            // Found a cycle: capture the ids from the first occurrence of `id`.
            const auto begin = std::find(stack.begin(), stack.end(), id);
            std::vector<std::string> cycle(begin, stack.end());
            result->cycles.push_back(cycle);
            for (const std::string& member : cycle) {
                result->errors.push_back({member, "dependency cycle detected involving '" + id + "'"});
            }
            return false;
        }
        if (stateIt != state.end() && stateIt->second == VisitState::Done) {
            return valid[id];
        }

        const ModManifest* manifest = index.at(id);
        state[id] = VisitState::Visiting;
        stack.push_back(id);

        bool ok = true;
        for (const Dependency& dependency : manifest->dependencies) {
            const auto providerIt = index.find(dependency.id);
            if (providerIt == index.end()) {
                result->errors.push_back({id, "missing dependency '" + dependency.id + "'"});
                ok = false;
                continue;
            }
            if (!satisfies(dependency.versionRange, providerIt->second->version)) {
                result->errors.push_back({id, "dependency '" + dependency.id + "' version " +
                                                  providerIt->second->version.toString() + " does not satisfy '" +
                                                  dependency.versionRange + "'"});
                ok = false;
                continue;
            }
            if (!visit(dependency.id)) {
                ok = false;
            }
        }

        stack.pop_back();
        state[id] = VisitState::Done;
        valid[id] = ok;
        if (ok) {
            order.push_back(id);
        }
        return ok;
    }
};

} // namespace

ResolveResult resolveMods(const std::vector<ModManifest>& manifests, const std::vector<std::string>& requestedIds) {
    ResolveResult result;

    std::unordered_map<std::string, const ModManifest*> index;
    for (const ModManifest& manifest : manifests) {
        const auto [it, inserted] = index.emplace(manifest.id, &manifest);
        if (!inserted) {
            result.conflicts.push_back("duplicate mod id '" + manifest.id + "'");
        }
    }

    std::vector<std::string> requested = requestedIds;
    if (requested.empty()) {
        for (const ModManifest& manifest : manifests) {
            if (std::find(requested.begin(), requested.end(), manifest.id) == requested.end()) {
                requested.push_back(manifest.id);
            }
        }
    }

    Resolver resolver{index};
    resolver.result = &result;

    for (const std::string& id : requested) {
        if (index.find(id) == index.end()) {
            result.errors.push_back({id, "requested mod '" + id + "' was not found"});
            result.disabled.push_back(id);
            continue;
        }
    }

    for (const std::string& id : requested) {
        if (index.find(id) == index.end()) {
            continue;
        }
        if (!resolver.visit(id)) {
            if (std::find(result.disabled.begin(), result.disabled.end(), id) == result.disabled.end()) {
                result.disabled.push_back(id);
            }
        }
    }

    // A dependency whose own provider chain failed may still appear disabled.
    for (const auto& [id, isValid] : resolver.valid) {
        if (!isValid && std::find(result.disabled.begin(), result.disabled.end(), id) == result.disabled.end()) {
            result.disabled.push_back(id);
        }
    }

    std::sort(result.disabled.begin(), result.disabled.end());
    result.order = resolver.order;
    return result;
}

} // namespace sim::modding
