// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "drive/Track.hpp"
#include "physics/PhysicsWorld.hpp"
#include "session/CarCatalog.hpp"
#include "session/CarLoader.hpp"
#include "session/ContentOwner.hpp"
#include "session/TrackCatalog.hpp"
#include "session/TrackLoader.hpp"
#include "vehicle/GroundQuery.hpp"
#include "vehicle/VehicleModel.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace {

namespace fs = std::filesystem;

constexpr double kDt = 1.0 / 1000.0;

// The official content and the example mod, mounted exactly like the game does.
fs::path sourceDir() {
    return fs::path(SIM_SOURCE_DIR);
}

// Which mod owns a virtual path: the base content or the example mod. This is
// the same job the app gives to ModManager::modIdFor in production.
sim::app::ContentOwner ownerFor(const sim::vfs::VirtualFileSystem& vfs) {
    return [&vfs](std::string_view path) -> std::string {
        const std::string real = vfs.realDirectoryOf(path);
        if (real.find("example_mod") != std::string::npos) {
            return "example.hatchback";
        }
        return "base.core";
    };
}

} // namespace

TEST_CASE("content: the official car and circuit load through the VFS") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));
    REQUIRE(vfs.mount(sourceDir() / "mods" / "example_mod"));

    const sim::app::CarLoadResult cars = sim::app::loadCars(vfs, ownerFor(vfs));
    for (const std::string& error : cars.errors) {
        MESSAGE("car error: " << error);
    }
    REQUIRE(cars.errors.empty());
    REQUIRE(cars.cars.size() == 2);
    // Every car carries a namespaced reference, never a bare name.
    CHECK(cars.cars[0].ref() == "base.core/voyager"); // sorted by reference
    CHECK(cars.cars[1].ref() == "example.hatchback/hatchback");
    CHECK(cars.cars[1].id == "hatchback");

    // The base car.toml must reproduce the compiled-in default bit for bit, so
    // moving the car into content changes nothing about how it drives.
    const sim::vehicle::VehicleSetup& expected = sim::vehicle::makeDefaultVehicleSetup();
    const sim::vehicle::VehicleSetup& actual = cars.cars[0].setup;
    CHECK(actual.mass == expected.mass);
    CHECK(actual.principalInertia == expected.principalInertia);
    CHECK(actual.bodyHalfExtents == expected.bodyHalfExtents);
    CHECK(actual.frontAxleZ == expected.frontAxleZ);
    CHECK(actual.rearAxleZ == expected.rearAxleZ);
    CHECK(actual.halfTrack == expected.halfTrack);
    CHECK(actual.restLength == expected.restLength);
    CHECK(actual.wheelRadius == expected.wheelRadius);
    CHECK(actual.wheelInertia == expected.wheelInertia);
    CHECK(actual.frontSpring.stiffness == expected.frontSpring.stiffness);
    CHECK(actual.rearSpring.stiffness == expected.rearSpring.stiffness);
    CHECK(actual.tyre.pDx1 == expected.tyre.pDx1);
    CHECK(actual.tyre.pDy1 == expected.tyre.pDy1);
    CHECK(actual.tyre.nominalLoad == expected.tyre.nominalLoad);
    CHECK(actual.maxBrakePressure == expected.maxBrakePressure);
    CHECK(actual.brakeBias.frontFraction == expected.brakeBias.frontFraction);
    CHECK(actual.gearbox.forwardRatios == expected.gearbox.forwardRatios);
    CHECK(actual.gearbox.reverseRatio == expected.gearbox.reverseRatio);
    CHECK(actual.finalDrive == expected.finalDrive);
    CHECK(actual.engine.config.idleSpeed == expected.engine.config.idleSpeed);
    CHECK(actual.engine.config.revLimit == expected.engine.config.revLimit);
    CHECK(actual.engine.map.speed == expected.engine.map.speed);
    CHECK(actual.engine.map.torque == expected.engine.map.torque);
    CHECK(actual.upshiftSpeed == expected.upshiftSpeed);
    CHECK(actual.downshiftSpeed == expected.downshiftSpeed);
    CHECK(actual.maxSteerAngle == expected.maxSteerAngle);
    CHECK(actual.dragCoefficient == expected.dragCoefficient);
    CHECK(actual.frontalArea == expected.frontalArea);
    CHECK(actual.ackermann.track == expected.ackermann.track);
    CHECK(actual.ackermann.wheelbase == expected.ackermann.wheelbase);

    const sim::app::TrackLoadResult tracks = sim::app::loadTracks(vfs, ownerFor(vfs));
    for (const std::string& error : tracks.errors) {
        MESSAGE("track error: " << error);
    }
    REQUIRE(tracks.errors.empty());
    REQUIRE(tracks.tracks.size() == 1);
    CHECK(tracks.tracks[0].ref() == "base.core/sunset_circuit");
    CHECK(tracks.tracks[0].lengthMeters > 100.0);
    CHECK(tracks.tracks[0].minimap.size() > 10);
    // The spec keeps the description the drive scene rebuilds the circuit from.
    CHECK(tracks.tracks[0].track.controlPoints.size() > 3);

    // The base track.toml must reproduce the compiled-in layout exactly, so the
    // circuit does not move when it comes from content.
    sim::app::TrackSpec expectedTrack;
    sim::app::fillTrackGeometry(expectedTrack, sim::app::drive::Track{});
    CHECK(tracks.tracks[0].lengthMeters == expectedTrack.lengthMeters);
    CHECK(tracks.tracks[0].widthMeters == expectedTrack.widthMeters);
    CHECK(tracks.tracks[0].minimap == expectedTrack.minimap);
    CHECK(tracks.tracks[0].aspect == expectedTrack.aspect);
}

TEST_CASE("content: the catalog resolves namespaced references and rejects ambiguity") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));
    REQUIRE(vfs.mount(sourceDir() / "mods" / "example_mod"));

    sim::app::CarCatalog& cars = sim::app::CarCatalog::instance();
    cars.reload(vfs, ownerFor(vfs));
    const sim::app::CarSpec* voyager = cars.find("base.core/voyager");
    REQUIRE(voyager != nullptr);
    CHECK(voyager->modId == "base.core");
    CHECK(voyager->id == "voyager");
    // A bare, unambiguous id still resolves (saved selections from v1).
    CHECK(cars.find("voyager") == voyager);
    CHECK(cars.find("base.core/does_not_exist") == nullptr);
    CHECK(cars.find("does_not_exist") == nullptr);

    sim::app::TrackCatalog& tracks = sim::app::TrackCatalog::instance();
    tracks.reload(vfs, ownerFor(vfs));
    REQUIRE(tracks.find("base.core/sunset_circuit") != nullptr);
    CHECK(tracks.find("base.core/sunset_circuit")->modId == "base.core");
    CHECK(tracks.find("missing") == nullptr);
}

TEST_CASE("content: a car discovered in a mod loads and drives") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));
    REQUIRE(vfs.mount(sourceDir() / "mods" / "example_mod"));

    const sim::app::CarLoadResult cars = sim::app::loadCars(vfs);
    REQUIRE(cars.errors.empty());

    const sim::app::CarSpec* hatchback = nullptr;
    for (const sim::app::CarSpec& car : cars.cars) {
        if (car.id == "hatchback") {
            hatchback = &car;
        }
    }
    REQUIRE(hatchback != nullptr);
    CHECK(hatchback->massKg == 980.0); // derived from the descriptor, not invented
    CHECK(hatchback->gearCount == 5);

    // Spawn it and drive: it must move under throttle and stay finite.
    sim::physics::PhysicsWorld world;
    sim::vehicle::FlatGroundQuery ground(0.0);
    sim::vehicle::VehicleModel& car = sim::vehicle::VehicleModel::spawn(world, hatchback->setup, ground);
    car.reset(sim::physics::Vec3(0.0), 0.0);

    sim::vehicle::VehicleInput input;
    input.throttle = 1.0;
    car.setInput(input);
    for (int i = 0; i < 3000; ++i) {
        car.step(kDt);
    }

    MESSAGE("hatchback: z " << car.position().z << " m, speed " << car.forwardSpeed() << " m/s");
    REQUIRE(car.chassis().isFiniteState());
    CHECK(car.forwardSpeed() > 3.0);
    CHECK(car.position().z < -2.0); // heading 0 faces -Z
}

TEST_CASE("content: an invalid car descriptor is rejected with a reason") {
    std::string error;

    // Out-of-range mass.
    const auto heavy =
        sim::app::parseCarSpec("id = \"lead_sled\"\n[chassis]\nmass = 90000.0\n", "cars/lead/car.toml", error);
    CHECK_FALSE(heavy.has_value());
    CHECK(error.find("mass") != std::string::npos);

    // Missing id.
    error.clear();
    const auto anonymous = sim::app::parseCarSpec("[chassis]\nmass = 1200.0\n", "cars/none/car.toml", error);
    CHECK_FALSE(anonymous.has_value());
    CHECK(error.find("id") != std::string::npos);

    // A valid descriptor parses.
    error.clear();
    const auto ok = sim::app::parseCarSpec("id = \"ok_car\"\n[chassis]\nmass = 1000.0\n", "cars/ok/car.toml", error);
    CHECK(ok.has_value());
    CHECK(error.empty());
}

TEST_CASE("content: a rejected value names the file, the line, the key and the value") {
    std::string error;
    const auto heavy =
        sim::app::parseCarSpec("id = \"lead_sled\"\n[chassis]\nmass = 90000.0\n", "cars/lead/car.toml", error);
    REQUIRE_FALSE(heavy.has_value());
    CHECK(error.find("cars/lead/car.toml") != std::string::npos);
    CHECK(error.find("line 3") != std::string::npos);
    CHECK(error.find("key 'mass'") != std::string::npos);
    CHECK(error.find("90000") != std::string::npos);
    CHECK(error.find("range") != std::string::npos);
    MESSAGE("rejected descriptor: " << error);
}

TEST_CASE("content: an invalid track descriptor is rejected with a reason") {
    std::string error;

    // Open circuits are not supported yet.
    const std::string open =
        "id = \"open_road\"\n[geometry]\nloop = false\ncontrol_points = [[0, 0], [0, 100], [100, 100]]\n";
    CHECK_FALSE(sim::app::parseTrackSpec(open, "tracks/open/track.toml", error).has_value());
    CHECK(error.find("loop") != std::string::npos);

    // A valid closed circuit parses.
    error.clear();
    const std::string closed =
        "id = \"triangle\"\n[geometry]\nloop = true\ncontrol_points = [[0, 0], [0, 200], [200, 100]]\n";
    const auto spec = sim::app::parseTrackSpec(closed, "tracks/triangle/track.toml", error);
    REQUIRE(spec.has_value());
    CHECK(error.empty());
    CHECK(spec->lengthMeters > 100.0);
}
