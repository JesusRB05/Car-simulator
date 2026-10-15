// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/CarCatalog.hpp"

#include <algorithm>
#include <cstddef>

namespace sim::app {
namespace {

// Fills the derived display values from the setup. Everything here is read
// straight out of the physics parameters, so the numbers on the cards are the
// numbers the car actually drives with.
void deriveSpecs(CarSpec& car) {
    const vehicle::VehicleSetup& setup = car.setup;

    car.massKg = setup.mass;
    car.gearCount = static_cast<int>(setup.gearbox.forwardRatios.size());
    car.wheelbaseM = setup.rearAxleZ - setup.frontAxleZ;
    car.trackWidthM = 2.0 * setup.halfTrack;
    car.wheelRadiusM = setup.wheelRadius;
    // The model is rear-wheel drive (VehicleModel sets driven = !front).
    car.drivetrain = "Rear-wheel drive";

    const vehicle::EngineTorqueMap& map = setup.engine.map;
    if (map.throttle.empty() || map.speed.empty()) {
        return;
    }
    const std::size_t columns = map.throttle.size();
    if (map.torque.size() < map.speed.size() * columns) {
        return;
    }
    double peakPowerW = 0.0;
    for (std::size_t i = 0; i < map.speed.size(); ++i) {
        // Wide-open-throttle column (throttle is increasing).
        const double torque = static_cast<double>(map.torque[i * columns + (columns - 1)]);
        car.peakTorqueNm = std::max(car.peakTorqueNm, torque);
        const double speed = static_cast<double>(map.speed[i]); // rad/s
        peakPowerW = std::max(peakPowerW, torque * speed);
    }
    car.peakPowerKw = peakPowerW / 1000.0;
}

CarSpec makeVoyager() {
    CarSpec car;
    car.id = "voyager";
    car.manufacturer = "Car Simulator contributors";
    car.model = "Voyager";
    car.category = "Sports car / RWD";
    car.description = "The base rear-wheel-drive test car: a ~1250 kg chassis on four virtual wheels with a 6-speed "
                      "sequential gearbox and an open differential. Defined in code and mirrored by "
                      "content/base/cars/voyager/car.toml.";
    car.bodyColor = glm::vec4(0.10f, 0.35f, 0.80f, 1.0f);
    car.setup = vehicle::makeDefaultVehicleSetup();
    deriveSpecs(car);
    return car;
}

} // namespace

CarCatalog::CarCatalog() {
    cars_.push_back(makeVoyager());
}

const CarCatalog& CarCatalog::instance() {
    static const CarCatalog catalog;
    return catalog;
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
