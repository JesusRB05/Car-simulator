// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/CarCatalog.hpp"

#include <utility>

#include "core/Log.hpp"
#include "session/CarLoader.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

// The compiled-in fallback car. It mirrors content/base/cars/voyager/car.toml
// and is only used if no descriptor loads (a broken install), so the game always
// has something to drive.
CarSpec makeVoyager() {
    CarSpec car;
    car.id = "voyager";
    car.manufacturer = "Car Simulator contributors";
    car.model = "Voyager";
    car.category = "Sports car / RWD";
    car.description = "The base rear-wheel-drive test car: a ~1250 kg chassis on four virtual wheels with a 6-speed "
                      "sequential gearbox and an open differential. Defined in content/base/cars/voyager/car.toml.";
    car.bodyColor = glm::vec4(0.10f, 0.35f, 0.80f, 1.0f);
    car.setup = vehicle::makeDefaultVehicleSetup();
    deriveCarSpec(car);
    return car;
}

} // namespace

CarCatalog::CarCatalog() {
    cars_.push_back(makeVoyager());
}

CarCatalog& CarCatalog::instance() {
    static CarCatalog catalog;
    return catalog;
}

void CarCatalog::reload(const vfs::VirtualFileSystem& vfs) {
    CarLoadResult result = loadCars(vfs);

    errors_ = std::move(result.errors);
    for (const std::string& error : errors_) {
        core::Log::get("content")->error("car rejected: {}", error);
    }

    if (result.cars.empty()) {
        core::Log::get("content")->warn("no car descriptor loaded; keeping the compiled-in Voyager");
        cars_.clear();
        cars_.push_back(makeVoyager());
        return;
    }

    cars_ = std::move(result.cars);
    core::Log::get("content")->info("loaded {} car(s) from content", cars_.size());
}

const CarSpec* CarCatalog::find(const std::string& id) const {
    for (const CarSpec& car : cars_) {
        if (car.id == id) {
            return &car;
        }
    }
    return nullptr;
}

} // namespace sim::app
