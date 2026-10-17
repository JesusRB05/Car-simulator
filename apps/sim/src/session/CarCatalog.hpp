// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "session/ContentOwner.hpp"
#include "vehicle/VehicleModel.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::app {

// One selectable car. The setup is the actual vehicle the physics spawns; the
// display fields are derived from the descriptor and are never invented.
//
// A car is identified by its **content reference** `<mod-id>/<content-id>`
// (for example `base.core/voyager`). `id` is the folder name inside the owning
// package; `modId` is the mod that provides it. Two mods may ship a car with
// the same folder name without colliding.
struct CarSpec {
    std::string modId; // owning mod, e.g. "base.core"
    std::string id;    // content id (folder name), e.g. "voyager"

    // The namespaced reference used everywhere a car is identified.
    std::string ref() const { return modId.empty() ? id : modId + "/" + id; }

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

// The set of cars the player can pick. Populated from the VFS: every
// `cars/<id>/car.toml` in the official content and in the enabled mods is
// loaded (see CarLoader). There is no compiled-in car: the official content is
// data (`base.core/voyager`). If nothing loads, the catalog is empty and the UI
// reports it.
class CarCatalog {
public:
    static CarCatalog& instance();

    // Rescans the VFS. Called once at start-up after the mods are mounted.
    // `owner` resolves which mod provides each descriptor (see ContentOwner).
    void reload(const vfs::VirtualFileSystem& vfs, const ContentOwner& owner = {});

    const std::vector<CarSpec>& cars() const { return cars_; }

    // Looks a car up by content reference (`base.core/voyager`) or, as a
    // fallback for saved selections, by a bare, unambiguous id.
    const CarSpec* find(const std::string& key) const;
    // The car used when a saved selection is missing or points at a gone car,
    // or nullptr when no car loaded at all.
    const CarSpec* defaultCar() const { return cars_.empty() ? nullptr : &cars_.front(); }

    // One entry per car descriptor that was rejected, with the reason.
    const std::vector<std::string>& errors() const { return errors_; }

private:
    CarCatalog();

    std::vector<CarSpec> cars_;
    std::vector<std::string> errors_;
};

} // namespace sim::app
