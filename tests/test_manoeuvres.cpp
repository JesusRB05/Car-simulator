// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// Vehicle-level manoeuvre suite (PHYSICS_PLAN Section 10.4). Each manoeuvre
// drives the assembled VehicleModel on a flat plane and checks the result
// against a closed-form oracle or a physically expected trend, so the integrated
// car - not just its parts - is validated.
#include <doctest/doctest.h>

#include <cmath>

#include "vehicle/GroundQuery.hpp"
#include "vehicle/VehicleModel.hpp"

using namespace sim::vehicle;
namespace physics = sim::physics;

namespace {

constexpr Real kDt = 1.0 / 1000.0;

// A car on a flat plane that owns its world and ground, so the model's pointers
// stay valid for the whole test.
struct FlatRig {
    physics::PhysicsWorld world;
    FlatGroundQuery ground;
    VehicleModel* car = nullptr;

    explicit FlatRig(Real friction = 1.0) : ground(0.0, friction) {
        car = &VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    }

    void hold(const VehicleInput& input, int steps) {
        car->setInput(input);
        for (int i = 0; i < steps; ++i) {
            car->step(kDt);
        }
    }

    // Full throttle in a straight line until the car passes `target`, returning
    // the speed it actually reached.
    Real launchTo(Real target, int maxSteps = 60000) {
        VehicleInput input;
        input.throttle = 1.0;
        car->setInput(input);
        for (int i = 0; i < maxSteps && car->forwardSpeed() < target; ++i) {
            car->step(kDt);
        }
        return car->forwardSpeed();
    }
};

constexpr Real lengthSquared(const physics::Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

// Steady-state turn with full steering: settle, then average the speed and the
// yaw rate over the last stretch.
struct Turn {
    Real speed = 0.0;
    Real radius = 0.0;
    Real kinematicRadius = 0.0; // wheelbase / tan(steer), the no-slip radius
};

Turn steadyTurn(FlatRig& rig, Real entrySpeed, Real throttle) {
    rig.launchTo(entrySpeed);
    VehicleInput input;
    input.steer = 1.0;
    input.throttle = throttle;
    rig.car->setInput(input);
    for (int i = 0; i < 3000; ++i) {
        rig.car->step(kDt);
    }

    Real speedSum = 0.0;
    Real yawSum = 0.0;
    const int samples = 1500;
    for (int i = 0; i < samples; ++i) {
        rig.car->step(kDt);
        speedSum += rig.car->forwardSpeed();
        yawSum += std::abs(rig.car->angularVelocity().y);
    }

    Turn turn;
    turn.speed = speedSum / samples;
    const Real yawRate = yawSum / samples;
    turn.radius = yawRate > 1e-9 ? turn.speed / yawRate : 1e9;
    const Real steer = std::abs(rig.car->steerAngle());
    turn.kinematicRadius = rig.car->setup().ackermann.wheelbase / std::tan(steer);
    return turn;
}

} // namespace

TEST_CASE("Manoeuvre: braking stops the car with a constant road-legal deceleration") {
    const auto stoppingDistance = [](Real entrySpeed) {
        FlatRig rig;
        rig.launchTo(entrySpeed);
        const physics::Vec3 start = rig.car->position();

        VehicleInput input;
        input.brake = 1.0;
        rig.car->setInput(input);
        int steps = 0;
        for (; steps < 10000 && rig.car->forwardSpeed() > 0.5; ++steps) {
            rig.car->step(kDt);
        }
        return std::sqrt(lengthSquared(rig.car->position() - start));
    };

    const Real slow = stoppingDistance(15.0);
    const Real fast = stoppingDistance(25.0);
    const Real deceleration = 15.0 * 15.0 / (2.0 * slow); // v^2 / 2d
    INFO("stopping distance: 15 m/s -> " << slow << " m, 25 m/s -> " << fast << " m (a = " << deceleration
                                         << " m/s^2)");

    // The brakes are sized below the tyre limit (3600 N*m total, DRIVE.md), so the
    // car stops at a comfortable road-car deceleration; aero and engine braking add
    // to the pad force at speed, so the distance grows a little faster than the
    // pure constant-force v^2/(2a) would.
    CHECK(deceleration > 4.0);
    CHECK(deceleration < 11.0);
    CHECK(fast > slow * 1.7);
    CHECK(fast < slow * 3.1);
}

TEST_CASE("Manoeuvre: 0-100 km/h acceleration is power limited and grip sensitive") {
    const auto timeTo100 = [](Real friction) {
        FlatRig rig(friction);
        VehicleInput input;
        input.throttle = 1.0;
        rig.car->setInput(input);
        const Real target = 100.0 / 3.6;
        int steps = 0;
        for (; steps < 60000 && rig.car->forwardSpeed() < target; ++steps) {
            rig.car->step(kDt);
        }
        return steps * kDt;
    };

    const Real highGrip = timeTo100(1.0);
    const Real lowGrip = timeTo100(0.35);
    INFO("0-100 km/h: grip 1.0 -> " << highGrip << " s, grip 0.35 -> " << lowGrip << " s");

    CHECK(highGrip > 2.0);
    CHECK(highGrip < 12.0);
    // Less grip means the driven wheels spin and the launch is slower.
    CHECK(lowGrip > highGrip);
}

TEST_CASE("Manoeuvre: coasting in gear slows the car (no driveline energy injection)") {
    FlatRig rig;
    rig.launchTo(15.0);

    VehicleInput input; // throttle closed, brakes off
    rig.car->setInput(input);
    for (int i = 0; i < 500; ++i) { // let the pedal release settle
        rig.car->step(kDt);
    }
    const Real settled = rig.car->forwardSpeed();

    for (int i = 0; i < 2500; ++i) { // 2.5 s of coasting
        rig.car->step(kDt);
    }
    const Real ended = rig.car->forwardSpeed();
    INFO("coasting: " << settled << " -> " << ended << " m/s, engine " << rig.car->engineSpeed() << " rad/s");

    // Engine braking and road load bleed speed; the driveline must never drive the
    // car forward with the throttle closed.
    CHECK(ended < settled);
    CHECK(ended > 6.0); // still rolling, no stall or lock
    CHECK(rig.car->engineSpeed() > 100.0);
}

TEST_CASE("Manoeuvre: braking in gear does not chatter the driveline") {
    FlatRig rig;
    rig.launchTo(20.0);

    VehicleInput input;
    input.brake = 1.0;
    rig.car->setInput(input);

    // Hold the brakes from 20 m/s and watch the driven axle: the clutch must not
    // slam between driving the wheels and engine-braking them every step.
    int flips = 0;
    Real lastSign = 0.0;
    Real peakDrive = 0.0;
    for (int i = 0; i < 4000 && rig.car->forwardSpeed() > 0.2; ++i) {
        rig.car->step(kDt);
        const Real drive = rig.car->corners()[2].driveTorque;
        peakDrive = std::max(peakDrive, std::abs(drive));
        const Real sign = drive > 1.0 ? 1.0 : (drive < -1.0 ? -1.0 : 0.0);
        if (sign != 0.0) {
            if (lastSign != 0.0 && sign != lastSign) {
                ++flips;
            }
            lastSign = sign;
        }
    }
    INFO("brake chatter: " << flips << " drive-torque sign flips, peak |drive| = " << peakDrive
                           << " N*m, v = " << rig.car->forwardSpeed());

    CHECK(flips <= 2);
    CHECK(peakDrive < 1200.0);
}

TEST_CASE("Manoeuvre: low-speed cornering matches the kinematic bicycle radius") {
    FlatRig rig;
    const Turn turn = steadyTurn(rig, 5.0, 0.25);

    INFO("v " << turn.speed << " m/s, radius " << turn.radius << " m, kinematic " << turn.kinematicRadius << " m");
    CHECK(turn.radius == doctest::Approx(turn.kinematicRadius).epsilon(0.35));
}

TEST_CASE("Manoeuvre: the car understeers as the cornering speed rises") {
    FlatRig slowRig;
    const Turn slow = steadyTurn(slowRig, 4.0, 0.2);

    FlatRig fastRig;
    const Turn fast = steadyTurn(fastRig, 14.0, 0.5);

    INFO("low  " << slow.speed << " m/s | radius " << slow.radius << " vs kinematic " << slow.kinematicRadius);
    INFO("high " << fast.speed << " m/s | radius " << fast.radius << " vs kinematic " << fast.kinematicRadius);

    // Same steering input, higher speed: the path radius grows (understeer), and
    // it exceeds the no-slip radius where the demanded lateral acceleration is
    // above what the tyres can deliver.
    CHECK(fast.radius > slow.radius);
    CHECK(fast.radius > fast.kinematicRadius * 1.1);
}
