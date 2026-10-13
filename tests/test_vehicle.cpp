// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <cmath>

// Every interface header is included so that it is compile-checked here.
#include "vehicle/IAerodynamics.hpp"
#include "vehicle/IBrakeSystem.hpp"
#include "vehicle/IDrivetrain.hpp"
#include "vehicle/ISuspension.hpp"
#include "vehicle/ITireModel.hpp"
#include "vehicle/IVehicleElectronics.hpp"
#include "vehicle/IVehicleModel.hpp"
#include "vehicle/PlaceholderVehicle.hpp"

using namespace sim::vehicle;

namespace {

constexpr Real kDt = 1.0 / 1000.0;

void run(IVehicleModel& model, const VehicleInput& input, int steps) {
    model.setInput(input);
    for (int i = 0; i < steps; ++i) {
        REQUIRE(model.step(kDt));
    }
}

} // namespace

TEST_CASE("PlaceholderVehicle at heading 0 faces and drives towards -Z") {
    PlaceholderVehicle car;
    car.reset(Vec3(0.0), 0.0);

    CHECK(car.forward().x == doctest::Approx(0.0));
    CHECK(car.forward().z == doctest::Approx(-1.0));
    CHECK(car.right().x == doctest::Approx(1.0));

    VehicleInput input;
    input.throttle = 1.0;
    run(car, input, 2000);

    CHECK(car.forwardSpeed() > 0.0);
    CHECK(car.position().z < -1.0);
    CHECK(std::abs(car.position().x) < 1e-9);
    CHECK(car.linearVelocity().z < 0.0);

    // The orientation quaternion maps body -Z onto the forward direction.
    const Vec3 rotated = car.orientation() * Vec3(0.0, 0.0, -1.0);
    CHECK(rotated.x == doctest::Approx(car.forward().x));
    CHECK(rotated.z == doctest::Approx(car.forward().z));
}

// Convention: positive steer (and positive heading/yaw rate about +Y) turns
// the vehicle to the LEFT, i.e. counter-clockwise seen from above (+Y), so a
// car starting at heading 0 drifts towards -X.
TEST_CASE("PlaceholderVehicle positive steer turns left (towards -X)") {
    PlaceholderVehicle car;
    car.reset(Vec3(0.0), 0.0);

    VehicleInput input;
    input.throttle = 1.0;
    input.steer = 1.0;
    run(car, input, 2000);

    CHECK(car.heading() > 0.0);
    CHECK(car.angularVelocity().y > 0.0);
    CHECK(car.position().x < 0.0);
    CHECK(car.position().z < 0.0);

    PlaceholderVehicle mirrored;
    mirrored.reset(Vec3(0.0), 0.0);
    input.steer = -1.0;
    run(mirrored, input, 2000);
    CHECK(mirrored.position().x > 0.0);
    CHECK(mirrored.position().x == doctest::Approx(-car.position().x));
}
