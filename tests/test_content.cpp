// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "drive/Track.hpp"
#include "modding/Composition.hpp"
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
    REQUIRE(cars.cars.size() == 3);
    // Every car carries a namespaced reference, never a bare name.
    CHECK(cars.cars[0].ref() == "base.core/halvard_gt3"); // sorted by reference
    CHECK(cars.cars[1].ref() == "base.core/voyager");
    CHECK(cars.cars[2].ref() == "example.hatchback/hatchback");
    CHECK(cars.cars[2].id == "hatchback");

    // The base car.toml must reproduce the compiled-in default bit for bit, so
    // moving the car into content changes nothing about how it drives.
    const sim::app::CarSpec* voyagerSpec = nullptr;
    for (const sim::app::CarSpec& car : cars.cars) {
        if (car.ref() == "base.core/voyager") {
            voyagerSpec = &car;
        }
    }
    REQUIRE(voyagerSpec != nullptr);
    const sim::vehicle::VehicleSetup& expected = sim::vehicle::makeDefaultVehicleSetup();
    const sim::vehicle::VehicleSetup& actual = voyagerSpec->setup;
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
    REQUIRE(tracks.tracks.size() == 2);
    // Sorted by reference: the modelled circuit before the procedural one.
    CHECK(tracks.tracks[0].ref() == "base.core/sierra_verde");
    CHECK(tracks.tracks[1].ref() == "base.core/sunset_circuit");

    // The modelled circuit resolves its glTF and its layout through the VFS and
    // keeps the authored start line, grid and sectors.
    const sim::app::TrackSpec& sierra = tracks.tracks[0];
    CHECK(sierra.modelPath == "tracks/sierra_verde/model/track.glb");
    CHECK(sierra.layoutPath == "tracks/sierra_verde/track.json");
    REQUIRE(sierra.layout.has_value());
    CHECK(sierra.layout->centreline.size() > 100);
    CHECK(sierra.layout->hasStart);
    CHECK(sierra.layout->grid.size() == 12);
    CHECK(sierra.layout->sectorDistances.size() >= 3);
    CHECK(sierra.layout->halfWidthMeters > 5.0);
    CHECK(sierra.lengthMeters > 2000.0);
    // The analytic fallback is derived from the layout, so the minimap and the
    // autopilot follow the real circuit.
    CHECK(sierra.track.controlPoints.size() > 100);
    CHECK(sierra.track.controlPoints.size() < sierra.layout->centreline.size());
    CHECK(sierra.minimap.size() > 10);

    const sim::app::TrackSpec& sunset = tracks.tracks[1];
    CHECK(sunset.lengthMeters > 100.0);
    CHECK(sunset.minimap.size() > 10);
    CHECK(sunset.modelPath.empty());
    CHECK(!sunset.layout.has_value());
    // The spec keeps the description the drive scene rebuilds the circuit from.
    CHECK(sunset.track.controlPoints.size() > 3);

    // The base track.toml must reproduce the compiled-in layout exactly, so the
    // circuit does not move when it comes from content.
    sim::app::TrackSpec expectedTrack;
    sim::app::fillTrackGeometry(expectedTrack, sim::app::drive::Track{});
    CHECK(sunset.lengthMeters == expectedTrack.lengthMeters);
    CHECK(sunset.widthMeters == expectedTrack.widthMeters);
    CHECK(sunset.minimap == expectedTrack.minimap);
    CHECK(sunset.aspect == expectedTrack.aspect);
}

TEST_CASE("content: the car model path and the liveries resolve through the VFS") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));
    REQUIRE(vfs.mount(sourceDir() / "mods" / "example_mod"));

    const sim::app::ContentOwner owner = ownerFor(vfs);
    const sim::modding::Composition composition = sim::modding::loadComposition(vfs, owner);
    for (const std::string& error : composition.errors) {
        MESSAGE("composition error: " << error);
    }
    REQUIRE(composition.errors.empty());

    sim::app::CarCatalog& cars = sim::app::CarCatalog::instance();
    cars.reload(vfs, owner, &composition);
    for (const std::string& error : cars.errors()) {
        MESSAGE("car error: " << error);
    }
    REQUIRE(cars.errors().empty());

    // `model` is relative to the descriptor's folder and is a glTF.
    const sim::app::CarSpec* gt3 = cars.find("base.core/halvard_gt3");
    REQUIRE(gt3 != nullptr);
    CHECK(gt3->modelPath == "cars/halvard_gt3/model/car.glb");
    CHECK(vfs.exists(gt3->modelPath));

    // Both shipped liveries attach to the car, with their maps already resolved.
    REQUIRE(gt3->skins.size() == 2);
    const sim::app::CarSkin* red = gt3->skin("red_black_8");
    REQUIRE(red != nullptr);
    CHECK(red->ref == "base.core/halvard_gt3/red_black_8");
    CHECK(red->name == "8 Red/Black");
    REQUIRE(red->textures.size() == 2);
    CHECK(red->textures[0].slot == "paint_livery.base_color");
    CHECK(red->textures[0].file == "skins/base.core/halvard_gt3/red_black_8/livery.png");
    CHECK(vfs.exists(red->textures[0].file));
    CHECK(red->textures[1].slot == "paint_livery.emissive");
    CHECK(vfs.exists(red->textures[1].file));
    CHECK(red->hasBodyColor);
    CHECK(red->bodyColor.r == doctest::Approx(0.08).epsilon(1e-4));
    CHECK(red->bodyColor.b == doctest::Approx(0.09).epsilon(1e-4));

    // The skins are sorted by display name, so the menu order is stable.
    CHECK(gt3->skins[0].id == "green_44");
    CHECK(gt3->skins[1].id == "red_black_8");
    CHECK(gt3->skin("does_not_exist") == nullptr);

    // A car without a glTF keeps the primitive fallback, and no skin.
    const sim::app::CarSpec* voyager = cars.find("base.core/voyager");
    REQUIRE(voyager != nullptr);
    CHECK(voyager->modelPath.empty());
    CHECK(voyager->skins.empty());
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

TEST_CASE("content: a patch changes a car without copying its file") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));
    const sim::app::ContentOwner owner = [](std::string_view) { return std::string("base.core"); };

    // Without the patch: the descriptor's own mass.
    const sim::app::CarLoadResult plain = sim::app::loadCars(vfs, owner);
    REQUIRE(plain.errors.empty());
    REQUIRE(plain.cars.size() == 2);
    const sim::app::CarSpec* stockCar = nullptr;
    for (const sim::app::CarSpec& car : plain.cars) {
        if (car.id == "voyager") {
            stockCar = &car;
        }
    }
    REQUIRE(stockCar != nullptr);
    const double stock = stockCar->massKg;

    // A patch adds 100 kg to the car, leaving its file untouched.
    std::string error;
    const auto patch = sim::modding::parsePatchText(
        "target = \"base.core/voyager\"\n[[patch]]\npath = \"chassis.mass\"\nop = \"add\"\nvalue = 100.0\n",
        "patches/heavy.toml", "ana.tune", error);
    REQUIRE(patch.has_value());
    sim::modding::Composition composition;
    composition.patches.push_back(*patch);

    const sim::app::CarLoadResult patched = sim::app::loadCars(vfs, owner, &composition);
    for (const std::string& message : patched.errors) {
        MESSAGE("patched car error: " << message);
    }
    REQUIRE(patched.errors.empty());
    REQUIRE(patched.cars.size() == 2);
    const sim::app::CarSpec* patchedCar = nullptr;
    for (const sim::app::CarSpec& car : patched.cars) {
        if (car.id == "voyager") {
            patchedCar = &car;
        }
    }
    REQUIRE(patchedCar != nullptr);
    CHECK(patchedCar->massKg == doctest::Approx(stock + 100.0));
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
