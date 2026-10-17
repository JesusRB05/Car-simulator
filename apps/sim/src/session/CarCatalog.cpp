// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include "session/CarCatalog.hpp"

#include <utility>

#include "core/Log.hpp"
#include "session/CarLoader.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {

CarCatalog::CarCatalog() = default;

CarCatalog& CarCatalog::instance() {
    static CarCatalog catalog;
    return catalog;
}

void CarCatalog::reload(const vfs::VirtualFileSystem& vfs, const ContentOwner& owner) {
    CarLoadResult result = loadCars(vfs, owner);

    errors_ = std::move(result.errors);
    for (const std::string& error : errors_) {
        core::Log::get("content")->error("car rejected: {}", error);
    }

    cars_ = std::move(result.cars);
    if (cars_.empty()) {
        core::Log::get("content")->error("no car descriptor loaded (is the base content mounted?)");
    } else {
        core::Log::get("content")->info("loaded {} car(s) from content", cars_.size());
    }
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
