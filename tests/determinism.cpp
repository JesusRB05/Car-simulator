// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// Cross-platform determinism probe. Builds a fixed physics scene, steps it a
// fixed number of times and prints the final `stateHash` in hexadecimal. CI
// runs it on Windows and Linux and fails if the hashes differ, which pins
// bit-for-bit reproducibility across platforms (compiler flags, libm, rounding).
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "physics/ForceElements.hpp"
#include "physics/Joints.hpp"
#include "physics/Math.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"
#include "vehicle/GroundQuery.hpp"
#include "vehicle/VehicleModel.hpp"

using namespace sim::physics;
using namespace sim::vehicle;

namespace {

constexpr Real kDt = 1.0 / 1000.0;
constexpr int kSteps = 20000;

RigidBody& makeDynamic(PhysicsWorld& world, Real mass, const Vec3& inertia, const Vec3& position) {
    RigidBody& body = world.createBody();
    body.setMass(mass);
    body.setInertiaPrincipal(inertia);
    body.position = position;
    body.updateCache();
    return body;
}

RigidBody& makeStatic(PhysicsWorld& world, const Vec3& position) {
    RigidBody& body = world.createBody();
    body.setType(BodyType::Static);
    body.setMass(0.0);
    body.position = position;
    body.updateCache();
    return body;
}

// A second scene that exercises the contact solver: a box and a sphere dropped
// onto a static ground box, all with collision enabled. Its hash is printed
// alongside the joint-scene hash so the cross-platform comparison also covers
// collision detection and response.
void buildContactScene(PhysicsWorld& world) {
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    RigidBody& ground = makeStatic(world, Vec3(0.0, -0.5, 0.0));
    ground.shape = Shape::box(Vec3(10.0, 0.5, 10.0));
    ground.collisionEnabled = true;
    ground.updateCache();

    RigidBody& box = makeDynamic(world, 1.0, Vec3(1.0), Vec3(0.0, 3.0, 0.0));
    box.shape = Shape::box(Vec3(0.5));
    box.collisionEnabled = true;
    box.updateCache();

    RigidBody& sphere = makeDynamic(world, 2.0, Vec3(0.2), Vec3(0.2, 4.0, 0.1));
    sphere.shape = Shape::sphere(0.5);
    sphere.collisionEnabled = true;
    sphere.updateCache();
}

// Kept identical to `buildDeterministicScene` in tests/test_physics.cpp so the
// probe exercises the same code path the determinism test asserts on.
void buildDeterministicScene(PhysicsWorld& world) {
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    RigidBody& pivot = makeStatic(world, Vec3(0.0));
    RigidBody& bob = makeDynamic(world, 1.0, Vec3(0.1), Vec3(0.3, -1.0, 0.0));
    DistanceConstraint& joint = world.createConstraint<DistanceConstraint>(pivot, bob);
    joint.localAnchorA = Vec3(0.0);
    joint.localAnchorB = Vec3(0.0);
    joint.distance = 1.0;

    RigidBody& free = makeDynamic(world, 0.5, Vec3(1.0, 2.0, 3.0), Vec3(0.0, 2.0, 0.0));
    free.angularVelocity = Vec3(0.2, 0.4, 0.7);
    PointSpringDamper& spring = world.createForceElement<PointSpringDamper>(&bob, nullptr);
    spring.localAnchorA = Vec3(0.0);
    spring.fixedAnchorB = Vec3(1.0, 0.0, 0.0);
    spring.restLength = 0.5;
    spring.stiffness = 20.0;
    spring.damping = 0.5;
}

std::uint64_t mixBits(std::uint64_t hash, double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    hash ^= bits;
    hash *= 1099511628211ULL;
    return hash;
}

// A full car manoeuvre: launch to 8 m/s, brake to a stop and pull away again.
// This is the "Manoeuvre: the car pulls away again after braking to a stop" case
// from tests/test_manoeuvres.cpp; hashing its end state pins the assembled
// vehicle (drivetrain, clutch, tyres, brakes), not just the raw physics world.
Real runManoeuvre(std::uint64_t& hash) {
    PhysicsWorld world;
    FlatGroundQuery ground(0.0, 1.0);
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);

    VehicleInput go;
    go.throttle = 1.0;
    car.setInput(go);
    for (int i = 0; i < 60000 && car.forwardSpeed() < 8.0; ++i) {
        car.step(kDt);
    }

    VehicleInput brake;
    brake.brake = 1.0;
    car.setInput(brake);
    for (int i = 0; i < 2500; ++i) {
        car.step(kDt);
    }

    VehicleInput again;
    again.throttle = 1.0;
    car.setInput(again);
    for (int i = 0; i < 2500; ++i) {
        car.step(kDt);
    }

    hash = world.stateHash();
    for (const VehicleCorner& corner : car.corners()) {
        hash = mixBits(hash, corner.spinWheel.spin);
        hash = mixBits(hash, corner.slipRatio);
    }
    hash = mixBits(hash, car.engineSpeed());
    hash = mixBits(hash, static_cast<double>(car.gear()));
    return car.forwardSpeed();
}

} // namespace

int main() {
    PhysicsWorld world;
    buildDeterministicScene(world);
    for (int i = 0; i < kSteps; ++i) {
        world.step(kDt);
    }
    std::printf("state_hash=%016" PRIx64 "\n", world.stateHash());

    PhysicsWorld contacts;
    buildContactScene(contacts);
    for (int i = 0; i < kSteps; ++i) {
        contacts.step(kDt);
    }
    std::printf("contact_hash=%016" PRIx64 "\n", contacts.stateHash());

    std::uint64_t manoeuvre = 0;
    const Real speed = runManoeuvre(manoeuvre);
    std::printf("manoeuvre_hash=%016" PRIx64 " speed=%.9f\n", manoeuvre, speed);
    return 0;
}
