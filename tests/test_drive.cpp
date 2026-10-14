// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "drive/Obstacles.hpp"
#include "drive/RaycastCar.hpp"
#include "drive/Track.hpp"
#include "drive/TrackGroundQuery.hpp"
#include "physics/PhysicsWorld.hpp"
#include "vehicle/VehicleModel.hpp"

using namespace sim::app::drive;
using namespace sim::physics;
using sim::physics::PhysicsWorld;

namespace {

void stepFor(PhysicsWorld& world, double seconds) {
    const int steps = static_cast<int>(seconds / world.fixedStep + 0.5);
    for (int i = 0; i < steps; ++i) {
        world.step(world.fixedStep);
    }
}

// Scripted drive on the circuit: launch, steer into a curve, then brake.
std::uint64_t scriptedRun(const Track& track, double& finalSpeed) {
    PhysicsWorld world;
    RaycastCar& car = RaycastCar::spawn(world, CarParams{}, track);
    car.reset(track.startPosition(), track.startHeading());
    for (int step = 0; step < 6000; ++step) {
        const double t = world.time();
        DriverInput input;
        input.throttle = t < 4.0 ? 1.0 : 0.0;
        input.steer = (t > 2.0 && t < 3.0) ? 0.6 : (t > 3.0 && t < 4.0 ? -0.4 : 0.0);
        input.brake = t > 4.5 ? 1.0 : 0.0;
        car.setInput(input);
        world.step(world.fixedStep);
    }
    finalSpeed = car.speed();
    return world.stateHash();
}

} // namespace

TEST_CASE("drive: the circuit is closed, smooth and does not overlap itself") {
    const Track track;
    const auto& samples = track.samples();
    REQUIRE(samples.size() > 100);
    CHECK(track.length() > 800.0);
    CHECK(track.length() < 1600.0);

    // Consecutive samples ~1 m apart, including the wrap-around.
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Vec3 d = samples[(i + 1) % samples.size()].position - samples[i].position;
        const double horizontal = std::sqrt(d.x * d.x + d.z * d.z);
        CHECK(horizontal == doctest::Approx(track.length() / static_cast<double>(samples.size())).epsilon(0.02));
    }

    // Parts of the track that are far apart along the centreline never come
    // closer than two full widths (asphalt + kerbs) plus a margin.
    const double band = 2.0 * (track.halfWidth() + track.kerbWidth()) + 2.0;
    double closest = 1e9;
    for (std::size_t i = 0; i < samples.size(); i += 2) {
        for (std::size_t j = i + 1; j < samples.size(); j += 2) {
            const double along = std::abs(samples[j].distance - samples[i].distance);
            if (std::min(along, track.length() - along) < 60.0) {
                continue;
            }
            const Vec3 d = samples[j].position - samples[i].position;
            closest = std::min(closest, std::sqrt(d.x * d.x + d.z * d.z));
        }
    }
    CHECK(closest > band);

    // Surfaces: the start line is asphalt, 30 m to the side is grass.
    const Vec3 start = track.samples()[track.startIndex()].position;
    CHECK(track.sample(start.x, start.z).surface == Surface::Asphalt);
    CHECK(track.sample(start.x + 30.0, start.z).surface == Surface::Grass);
    CHECK(track.sample(start.x + 30.0, start.z).friction < track.sample(start.x, start.z).friction);

    // The analytic normal matches the height function.
    const double x = 70.0;
    const double z = -30.0;
    const double e = 1e-4;
    const double gx = (Track::terrainHeight(x + e, z) - Track::terrainHeight(x - e, z)) / (2.0 * e);
    const double gz = (Track::terrainHeight(x, z + e) - Track::terrainHeight(x, z - e)) / (2.0 * e);
    const Vec3 n = Track::terrainNormal(x, z);
    CHECK(-n.x / n.y == doctest::Approx(gx).epsilon(1e-5));
    CHECK(-n.z / n.y == doctest::Approx(gz).epsilon(1e-5));

    // The car starts facing -Z on the main straight.
    CHECK(std::abs(track.startHeading()) < 0.03);
}

TEST_CASE("drive: a car at rest settles to the static ride height") {
    PhysicsWorld world;
    const FlatGround ground(0.0);
    const CarParams params;
    RaycastCar& car = RaycastCar::spawn(world, params, ground);
    car.reset(Vec3(0.0), 0.0);
    // Start slightly high so it actually has to settle.
    car.chassis().position.y += 0.05;

    stepFor(world, 4.0);

    const double weight = params.mass * -world.gravity.y;
    double springSum = 0.0;
    for (const WheelState& wheel : car.wheels()) {
        CHECK(wheel.contact);
        springSum += wheel.springForce;
    }
    CHECK(std::abs(springSum - weight) / weight < 0.01);
    CHECK(car.chassis().position.y == doctest::Approx(car.staticRideHeight(-world.gravity.y)).epsilon(0.005));
    CHECK(glm::length(car.chassis().linearVelocity) < 1e-3);
    CHECK(glm::length(car.chassis().angularVelocity) < 1e-3);
    // The car does not creep.
    CHECK(std::abs(car.chassis().position.x) < 1e-3);
    CHECK(std::abs(car.chassis().position.z) < 1e-3);
}

TEST_CASE("drive: full throttle in a straight line accelerates towards -Z") {
    PhysicsWorld world;
    const FlatGround ground(0.0);
    RaycastCar& car = RaycastCar::spawn(world, CarParams{}, ground);
    car.reset(Vec3(0.0), 0.0);

    DriverInput input;
    input.throttle = 1.0;
    car.setInput(input);
    stepFor(world, 5.0);

    const auto& body = car.chassis();
    CHECK(body.position.z < -25.0);
    CHECK(body.linearVelocity.z < -12.0);
    CHECK(car.forwardSpeed() > 12.0);
    CHECK(std::abs(body.position.x) < 0.1);
    CHECK(std::abs(body.linearVelocity.x) < 0.05);
    CHECK(car.gear() >= 2);
    CHECK(body.isFiniteState());

    // Braking brings it back to rest, and holding the brake then engages reverse.
    input = DriverInput{};
    input.brake = 1.0;
    car.setInput(input);
    stepFor(world, 6.0);
    CHECK(car.gear() == -1);
    CHECK(car.forwardSpeed() < 0.0); // reversing
}

TEST_CASE("drive: the car is stopped by a static barrier") {
    PhysicsWorld world;
    const FlatGround ground(0.0);
    RaycastCar& car = RaycastCar::spawn(world, CarParams{}, ground);
    car.reset(Vec3(0.0), 0.0); // facing -Z

    // A wide barrier 30 m ahead, tall enough that the chassis box (0.14..0.84 m)
    // overlaps it. Both bodies opt into collision.
    RigidBody& barrier = world.createBody();
    barrier.name = "barrier";
    barrier.setType(BodyType::Static);
    barrier.shape = Shape::box(Vec3(3.0, 0.5, 0.25));
    barrier.position = Vec3(0.0, 0.5, -30.0);
    barrier.collisionEnabled = true;
    barrier.setMass(0.0);
    barrier.updateCache();

    DriverInput input;
    input.throttle = 1.0;
    car.setInput(input);
    stepFor(world, 6.0);

    REQUIRE(car.chassis().isFiniteState());
    // The barrier's near face is at z = -29.75 and the nose is 2.15 m ahead of
    // the CoM, so the CoM must stay well short of the barrier instead of
    // tunnelling through it.
    CHECK(car.chassis().position.z > -28.0);
    CHECK(car.forwardSpeed() < 2.0);
}

TEST_CASE("drive: the obstacle layout lines the circuit and keeps the asphalt clear") {
    const Track track;
    const std::vector<Obstacle> obstacles = buildObstacleLayout(track);
    REQUIRE(obstacles.size() > 40);

    const double clear = track.halfWidth() + track.kerbWidth();
    bool sawBarrier = false;
    bool sawYard = false;
    for (const Obstacle& obstacle : obstacles) {
        const Track::Projection projection = track.project(obstacle.position.x, obstacle.position.z);
        if (projection.found) {
            // No obstacle sits on the asphalt + kerb band around the centreline.
            CHECK(std::abs(projection.lateral) > clear);
        }
        sawBarrier = sawBarrier || obstacle.kind == "barrier";
        sawYard = sawYard || obstacle.kind == "cone";
    }
    CHECK(sawBarrier);
    CHECK(sawYard);
}

TEST_CASE("drive: the adapter exposes the circuit surface to the vehicle") {
    const Track track;
    TrackGroundQuery query(track);

    const Vec3 onAsphalt = track.samples()[track.startIndex()].position;
    const GroundSample surface = track.sample(onAsphalt.x, onAsphalt.z);
    const sim::vehicle::GroundSample viaAdapter = query.sample(onAsphalt.x, onAsphalt.z);
    CHECK(viaAdapter.height == doctest::Approx(surface.height).epsilon(1e-12));
    CHECK(viaAdapter.normal.y == doctest::Approx(surface.normal.y).epsilon(1e-12));
    CHECK(viaAdapter.friction == doctest::Approx(surface.friction).epsilon(1e-12));
    CHECK(viaAdapter.rollingResistance == doctest::Approx(surface.rollingResistance).epsilon(1e-12));

    // Far from the track the surface is grass: the friction the model reads matches.
    const Vec3 onGrass(onAsphalt.x + 30.0, 0.0, onAsphalt.z);
    CHECK(query.sample(onGrass.x, onGrass.z).friction ==
          doctest::Approx(track.sample(onGrass.x, onGrass.z).friction).epsilon(1e-12));
    CHECK(query.sample(onGrass.x, onGrass.z).friction < query.sample(onAsphalt.x, onAsphalt.z).friction);
}

TEST_CASE("drive: an assembled VehicleModel settles on the circuit through the adapter") {
    const Track track;
    PhysicsWorld world;
    TrackGroundQuery ground(track);
    sim::vehicle::VehicleModel& car =
        sim::vehicle::VehicleModel::spawn(world, sim::vehicle::makeDefaultVehicleSetup(), ground);
    car.reset(track.startPosition(), track.startHeading());

    // VehicleModel owns the step: the world advances inside car.step().
    for (int i = 0; i < 3000; ++i) {
        car.step(world.fixedStep);
    }

    REQUIRE(car.chassis().isFiniteState());
    for (const sim::vehicle::VehicleCorner& corner : car.corners()) {
        CHECK(corner.load > 0.0);
    }
    CHECK(car.forwardSpeed() > -0.5);
    CHECK(car.forwardSpeed() < 0.5);
}

TEST_CASE("drive: two identical scripted runs produce the same state hash") {
    const Track track;
    double speedA = 0.0;
    double speedB = 0.0;
    const std::uint64_t a = scriptedRun(track, speedA);
    const std::uint64_t b = scriptedRun(track, speedB);
    CHECK(a == b);
    CHECK(speedA == speedB);
    CHECK(std::isfinite(speedA));
}

TEST_CASE("drive: a simple autopilot laps the circuit on the asphalt without rolling over") {
    const Track track;
    PhysicsWorld world;
    const CarParams params;
    RaycastCar& car = RaycastCar::spawn(world, params, track);
    car.reset(track.startPosition(), track.startHeading());

    const auto& samples = track.samples();
    const double length = track.length();
    const std::size_t count = samples.size();
    const double spacing = length / static_cast<double>(count);

    double progress = 0.0;
    double previousAlong = track.project(track.startPosition().x, track.startPosition().z).along;
    double maxLateral = 0.0;
    double minUp = 1.0;
    double topSpeed = 0.0;
    int steps = 0;
    bool valid = true;
    while (progress < length && steps < 180000) {
        const auto& body = car.chassis();
        const Track::Projection here = track.project(body.position.x, body.position.z);
        if (!here.found) {
            valid = false;
            break;
        }
        double delta = here.along - previousAlong;
        if (delta < -0.5 * length) {
            delta += length;
        } else if (delta > 0.5 * length) {
            delta -= length;
        }
        progress += delta;
        previousAlong = here.along;
        maxLateral = std::max(maxLateral, std::abs(here.lateral));
        minUp = std::min(minUp, body.localToWorldDirection(Vec3(0.0, 1.0, 0.0)).y);
        topSpeed = std::max(topSpeed, car.speed());

        // Pure pursuit on the centreline.
        const double v = car.forwardSpeed();
        const double lookahead = 8.0 + 0.6 * std::abs(v);
        const std::size_t target = (here.segment + static_cast<std::size_t>(lookahead / spacing)) % count;
        const Vec3 toTarget = samples[target].position - body.position;
        const Vec3 forward = body.localToWorldDirection(Vec3(0.0, 0.0, -1.0));
        const Vec3 left = body.localToWorldDirection(Vec3(-1.0, 0.0, 0.0));
        const double alpha = std::atan2(glm::dot(toTarget, left), glm::dot(toTarget, forward));
        const double wheelAngle = std::atan(2.0 * params.wheelbase() * std::sin(alpha) / lookahead);
        const double limit =
            std::max(params.minSteerAngle, params.maxSteerAngle / (1.0 + std::abs(v) / params.steerSpeedScale));

        // Speed from the tightest curvature in the next 60 m.
        double curvature = 1e-3;
        for (std::size_t k = 0; k < static_cast<std::size_t>(60.0 / spacing); ++k) {
            curvature = std::max(curvature, std::abs(samples[(here.segment + k) % count].curvature));
        }
        const double targetSpeed = std::min(32.0, std::sqrt(0.7 * 9.81 / curvature));

        DriverInput input;
        input.steer = std::clamp(wheelAngle / limit, -1.0, 1.0);
        input.throttle = v < targetSpeed - 1.0 ? 1.0 : 0.0;
        input.brake = v > targetSpeed + 1.5 ? 1.0 : 0.0;
        car.setInput(input);

        world.step(world.fixedStep);
        ++steps;
        if (!car.chassis().isFiniteState()) {
            valid = false;
            break;
        }
    }

    MESSAGE("autopilot lap: " << world.time() << " s, top speed " << topSpeed * 3.6 << " km/h, max |lateral| "
                              << maxLateral << " m, min up.y " << minUp);
    REQUIRE(valid);
    CHECK(progress >= length);
    CHECK(maxLateral < track.halfWidth());
    CHECK(minUp > 0.95);
    CHECK(topSpeed > 25.0);
}
