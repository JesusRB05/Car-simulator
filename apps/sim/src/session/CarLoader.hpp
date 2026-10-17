// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "session/CarCatalog.hpp"
#include "session/ContentOwner.hpp"

namespace sim::modding {
struct Composition;
}

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::app {

// Result of scanning the VFS for car descriptors.
struct CarLoadResult {
    std::vector<CarSpec> cars;
    // One entry per rejected descriptor: "<virtual path>: <message>". A bad car
    // never aborts the scan; it is skipped and reported.
    std::vector<std::string> errors;
};

// Fills the derived display fields (mass, gear count, peak torque/power, ...)
// from the setup, so the numbers on the cards are the numbers the car drives
// with. Shared by the loader and the compiled-in fallback car.
void deriveCarSpec(CarSpec& car);

// Parses one car descriptor (the text of a car.toml). Returns std::nullopt and
// fills `error` when the document is malformed or a value is out of range. All
// values are SI: metres, kilograms, radians and seconds; speeds are rad/s.
std::optional<CarSpec> parseCarSpec(std::string_view text, std::string_view sourcePath, std::string& error);

// Loads every `cars/<id>/car.toml` reachable through the VFS (official content
// and mods alike). The optional `owner` gives each car its owning mod so its
// content reference is `<mod-id>/<id>`. When `composition` is given, any patch
// targeting a car's reference is applied to its descriptor before validation.
// Cars are returned sorted by reference. Invalid cars are skipped with an error;
// the rest still load.
CarLoadResult loadCars(const vfs::VirtualFileSystem& vfs, const ContentOwner& owner = {},
                       const modding::Composition* composition = nullptr);

} // namespace sim::app
