// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include "session/CarCatalog.hpp"

#include <algorithm>
#include <utility>

#include "core/Log.hpp"
#include "modding/Composition.hpp"
#include "session/CarLoader.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {

namespace {

// Copies the scanned liveries onto the cars they name. A skin whose car is not
// registered is reported once and dropped: it may belong to a mod that is off.
void attachSkins(const modding::Composition& composition, std::vector<CarSpec>& cars,
                 std::vector<std::string>& errors) {
    for (CarSpec& candidate : cars) {
        candidate.skins.clear();
    }
    for (const modding::Skin& skin : composition.skins) {
        auto target = std::find_if(cars.begin(), cars.end(),
                                   [&](const CarSpec& candidate) { return candidate.ref() == skin.car; });
        if (target == cars.end()) {
            errors.push_back(skin.source + ": the car '" + skin.car + "' is not registered");
            continue;
        }
        CarSkin entry;
        entry.ref = skin.ref;
        entry.id = skin.ref.substr(skin.car.size() + 1);
        entry.name = skin.name.empty() ? entry.id : skin.name;
        entry.description = skin.description;
        for (const modding::SkinTexture& texture : skin.textures) {
            entry.textures.push_back({texture.slot, texture.file});
        }
        if (!skin.bodyColor.empty()) {
            entry.hasBodyColor = true;
            for (std::size_t i = 0; i < skin.bodyColor.size(); ++i) {
                entry.bodyColor[static_cast<glm::length_t>(i)] = static_cast<float>(skin.bodyColor[i]);
            }
        }
        target->skins.push_back(std::move(entry));
    }
    for (CarSpec& candidate : cars) {
        std::sort(candidate.skins.begin(), candidate.skins.end(),
                  [](const CarSkin& a, const CarSkin& b) { return a.name < b.name; });
    }
}

} // namespace

CarCatalog::CarCatalog() = default;

CarCatalog& CarCatalog::instance() {
    static CarCatalog catalog;
    return catalog;
}

void CarCatalog::reload(const vfs::VirtualFileSystem& vfs, const ContentOwner& owner,
                        const modding::Composition* composition) {
    CarLoadResult result = loadCars(vfs, owner, composition);

    errors_ = std::move(result.errors);
    cars_ = std::move(result.cars);
    if (composition != nullptr) {
        attachSkins(*composition, cars_, errors_);
    }
    for (const std::string& error : errors_) {
        core::Log::get("content")->error("car rejected: {}", error);
    }

    if (cars_.empty()) {
        core::Log::get("content")->error("no car descriptor loaded (is the base content mounted?)");
    } else {
        core::Log::get("content")->info("loaded {} car(s) from content", cars_.size());
    }
}

const CarSkin* CarSpec::skin(std::string_view id) const {
    for (const CarSkin& entry : skins) {
        if (entry.id == id || entry.ref == id) {
            return &entry;
        }
    }
    return nullptr;
}

const CarSpec* CarCatalog::find(const std::string& key) const {
    // Exact content reference first.
    for (const CarSpec& car : cars_) {
        if (car.ref() == key) {
            return &car;
        }
    }
    // Fallback for a saved bare id: only unambiguous.
    const CarSpec* match = nullptr;
    for (const CarSpec& car : cars_) {
        if (car.id == key) {
            if (match != nullptr) {
                return nullptr; // ambiguous, refuse to guess
            }
            match = &car;
        }
    }
    return match;
}

} // namespace sim::app
