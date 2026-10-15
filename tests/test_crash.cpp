// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// Collision-physics matrix: the assembled car against static barriers from the
// front, the rear, the side and obliquely, into a corner, and over kerbs and
// speed bumps. The shape-collision system must stop the car without letting it
// tunnel, and the raycast wheels must ride raised road furniture.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "physics/PhysicsWorld.hpp"
#include "vehicle/GroundQuery.hpp"
#include "vehicle/VehicleModel.hpp"

using namespace sim::vehicle;
namespace physics = sim::physics;
using physics::BodyType;
using physics::PhysicsWorld;
using physics::RigidBody;
using physics::Shape;

namespace {

constexpr Real kDt = 1.0 / 1000.0;
constexpr Real kRadToDeg = 57.295779513082320876;

RigidBody& addStaticBox(PhysicsWorld& world, const Vec3& halfExtents, const Vec3& position,
                        const Quat& orientation = Quat(1.0, 0.0, 0.0, 0.0)) {
    RigidBody& body = world.createBody();
    body.name = "barrier";
    body.setType(BodyType::Static);
    body.shape = Shape::box(halfExtents);
    body.position = position;
    body.orientation = orientation;
    body.collisionEnabled = true;
    body.setMass(0.0);
    body.updateCache();
    return body;
}

// Ground with a raised strip (kerb) on the driver's right beyond a start line, so
// the right wheels mount it while the left stay low.
class KerbGround final : public IGroundQuery {
public:
    GroundSample sample(Real x, Real z) const override {
        GroundSample result;
        result.height = (x >= 0.5 && z <= -12.0) ? 0.04 : 0.0;
        result.normal = Vec3(0.0, 1.0, 0.0);
        result.friction = 1.0;
        return result;
    }
};

// A full-width step (speed bump) across the path at z = -15.
class BumpGround final : public IGroundQuery {
public:
    GroundSample sample(Real, Real z) const override {
        GroundSample result;
        result.height = z <= -15.0 ? 0.06 : 0.0;
        result.normal = Vec3(0.0, 1.0, 0.0);
        result.friction = 1.0;
        return result;
    }
};

} // namespace

TEST_CASE("collision: a head-on barrier stops the car without tunnelling") {
    PhysicsWorld world;
    FlatGroundQuery ground(0.0);
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    car.reset(Vec3(0.0), 0.0);

    // Near face at z = -29.75; the nose is 2.15 m ahead of the CoM.
    addStaticBox(world, Vec3(3.0, 0.5, 0.25), Vec3(0.0, 0.5, -30.0));

    VehicleInput input;
    input.throttle = 1.0;
    car.setInput(input);
    for (int i = 0; i < 6000; ++i) {
        car.step(kDt);
    }
    MESSAGE("head-on: CoM z " << car.position().z << " m, speed " << car.forwardSpeed() << " m/s");
    REQUIRE(car.chassis().isFiniteState());
    CHECK(car.position().z > -28.0);
    CHECK(car.forwardSpeed() < 2.0);
}

TEST_CASE("collision: an oblique barrier deflects the car instead of letting it through") {
    PhysicsWorld world;
    FlatGroundQuery ground(0.0);
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    car.reset(Vec3(0.0), 0.0);

    addStaticBox(world, Vec3(4.0, 0.5, 0.25), Vec3(0.0, 0.5, -30.0),
                 physics::quaternionFromAxisAngle(Vec3(0.0, 1.0, 0.0), 0.5));

    VehicleInput input;
    input.throttle = 1.0;
    car.setInput(input);
    for (int i = 0; i < 6000; ++i) {
        car.step(kDt);
    }
    // Deflected sideways but still in front of the barrier plane.
    const Real alongWall = car.position().z;
    MESSAGE("oblique: CoM " << car.position().x << ", " << car.position().z << " m, speed " << car.forwardSpeed());
    REQUIRE(car.chassis().isFiniteState());
    CHECK(alongWall > -30.5);                  // did not pass the barrier
    CHECK(std::abs(car.position().x) > 0.15);  // was pushed sideways
    CHECK(std::abs(car.forwardSpeed()) < 3.0); // and stopped
}

TEST_CASE("collision: backing into a barrier stops the car") {
    PhysicsWorld world;
    FlatGroundQuery ground(0.0);
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    car.reset(Vec3(0.0), 0.0); // facing -Z, so the barrier behind is at +z

    // Near face at z = 7.75; the tail is 2.15 m behind the CoM.
    addStaticBox(world, Vec3(3.0, 0.5, 0.25), Vec3(0.0, 0.5, 8.0));

    // Hold the brake while stopped to select reverse, then back into the wall.
    VehicleInput brake;
    brake.brake = 1.0;
    car.setInput(brake);
    for (int i = 0; i < 400; ++i) {
        car.step(kDt);
    }
    VehicleInput reverse;
    reverse.throttle = 0.4; // below the throttle that would flip back to first
    car.setInput(reverse);
    for (int i = 0; i < 6000; ++i) {
        car.step(kDt);
    }
    MESSAGE("backing: CoM z " << car.position().z << " m, speed " << car.forwardSpeed() << " m/s");
    REQUIRE(car.chassis().isFiniteState());
    // The tail must stay short of the near face at z = 7.75 (no tunnelling).
    CHECK(car.position().z + 2.15 < 8.5);
    CHECK(std::abs(car.chassis().linearVelocity.z) < 2.0);
}

TEST_CASE("collision: a side impact pushes the car away from the wall") {
    PhysicsWorld world;
    FlatGroundQuery ground(0.0);
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    car.reset(Vec3(0.0), 0.0);

    // Near face at x = -1.75, so the body (half-width 0.88) reaches it at x=-0.87.
    addStaticBox(world, Vec3(0.25, 0.5, 3.0), Vec3(-2.0, 0.5, 0.0));

    // Slide the car sideways into the wall.
    car.chassis().linearVelocity = Vec3(-8.0, 0.0, 0.0);
    VehicleInput input;
    car.setInput(input);
    for (int i = 0; i < 2000; ++i) {
        car.step(kDt);
    }
    MESSAGE("side: CoM x " << car.position().x << " m, vx " << car.chassis().linearVelocity.x);
    REQUIRE(car.chassis().isFiniteState());
    CHECK(car.position().x > -2.1);               // did not pass through the wall centre
    CHECK(car.chassis().linearVelocity.x > -1.0); // the wall absorbed the slide
}

TEST_CASE("collision: an inside corner of two barriers contains the car") {
    PhysicsWorld world;
    FlatGroundQuery ground(0.0);
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);

    // An inside corner at (-6, -20): a wall across the path and a wall to the
    // left meeting there.
    addStaticBox(world, Vec3(10.0, 0.5, 0.25), Vec3(-6.0, 0.5, -20.0));
    addStaticBox(world, Vec3(0.25, 0.5, 12.0), Vec3(-6.0, 0.5, -8.0));

    // Approach the corner along its diagonal (forward = -sin h, 0, -cos h).
    const Real heading = 0.411;
    car.reset(Vec3(1.2, 0.0, -3.5), heading);
    VehicleInput input;
    input.throttle = 1.0;
    car.setInput(input);
    for (int i = 0; i < 4000; ++i) {
        car.step(kDt);
    }
    // Settle in the corner.
    VehicleInput stop;
    stop.brake = 1.0;
    car.setInput(stop);
    for (int i = 0; i < 3000; ++i) {
        car.step(kDt);
    }
    MESSAGE("corner: CoM " << car.position().x << ", " << car.position().z << " m, speed " << car.forwardSpeed());
    REQUIRE(car.chassis().isFiniteState());
    CHECK(car.position().x > -6.6);  // did not pass the left wall
    CHECK(car.position().z > -20.6); // did not pass the far wall
    CHECK(car.forwardSpeed() < 5.0);
}

TEST_CASE("collision: the car rides up a kerb on one side") {
    KerbGround ground;
    PhysicsWorld world;
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    car.reset(Vec3(0.0), 0.0);

    Real maxRoll = 0.0;
    VehicleInput input;
    input.throttle = 0.6;
    car.setInput(input);
    for (int i = 0; i < 6000; ++i) {
        car.step(kDt);
        const Vec3 right = car.chassis().localToWorldDirection(Vec3(1.0, 0.0, 0.0));
        maxRoll = std::max(maxRoll, right.y);
    }
    MESSAGE("kerb: CoM z " << car.position().z << " m, max right.y " << maxRoll << " (roll "
                           << std::asin(std::clamp(maxRoll, -1.0, 1.0)) * kRadToDeg << " deg)");
    REQUIRE(car.chassis().isFiniteState());
    CHECK(car.position().z < -20.0); // it crossed onto the kerb
    CHECK(maxRoll > 0.01);           // the right side lifted
    CHECK(car.forwardSpeed() > 3.0); // and it kept moving
}

TEST_CASE("collision: a speed bump upsets the car but does not break it") {
    BumpGround ground;
    PhysicsWorld world;
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    car.reset(Vec3(0.0), 0.0);

    Real maxCompression = 0.0;
    VehicleInput input;
    input.throttle = 1.0;
    car.setInput(input);
    for (int i = 0; i < 6000; ++i) {
        car.step(kDt);
        REQUIRE(car.chassis().isFiniteState());
        maxCompression = std::max(maxCompression, car.corner(0).compression);
    }
    MESSAGE("bump: CoM z " << car.position().z << " m, max front compression " << maxCompression << " m, speed "
                           << car.forwardSpeed() << " m/s");
    CHECK(car.position().z < -25.0); // cleared the bump
    CHECK(maxCompression > 0.0);     // the suspension moved
    CHECK(car.forwardSpeed() > 5.0);
}
