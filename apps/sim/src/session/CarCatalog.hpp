// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "vehicle/VehicleModel.hpp"

namespace sim::app {

// One selectable car. The setup is the actual vehicle the physics spawns; the
// display fields are derived from it (or from content/base/cars/<id>) and are
// never invented. Cars are registered in CarCatalog; adding one is a matter of
// pushing a CarSpec built from a VehicleSetup plus its metadata.
struct CarSpec {
    std::string id;
    std::string manufacturer;
    std::string model;
    std::string category;
    std::string description;

    // The physics setup actually used when the car is spawned.
    vehicle::VehicleSetup setup{};

    // Visual identity for the garage preview and cards.
    glm::vec4 bodyColor{0.10f, 0.35f, 0.80f, 1.0f};

    // Progression hook. No progression system exists yet, so every registered
    // car is unlocked; the flag is here so a future career mode can fill it in.
    bool locked = false;
    std::string unlockNote;

    // Derived, real values shown in the garage (mass and geometry from the
    // setup, peak torque/power from the wide-open-throttle torque map).
    double massKg = 0.0;
    double peakTorqueNm = 0.0;
    double peakPowerKw = 0.0;
    int gearCount = 0;
    std::string drivetrain = "Unknown";
    double wheelbaseM = 0.0;
    double trackWidthM = 0.0;
    double wheelRadiusM = 0.0;
};

// The set of cars the player can pick. Built lazily from the compiled-in
// vehicles; there is no data-driven loader yet (content/base/cars/*/car.toml is
// documentation only), so this is the single source of truth.
class CarCatalog {
public:
    static const CarCatalog& instance();

    const std::vector<CarSpec>& cars() const { return cars_; }
    const CarSpec* find(const std::string& id) const;
    // The car used when a saved selection is missing or points at a gone car.
    const CarSpec& defaultCar() const { return cars_.front(); }

private:
    CarCatalog();

    std::vector<CarSpec> cars_;
};

} // namespace sim::app
