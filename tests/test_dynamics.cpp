// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// Physics-dynamics matrix (PHYSICS_PLAN Phase 3): does the raycast car follow an
// undulating surface, launch and land off a ramp like a projectile, drift
// coherently, and brake in a straight line, in a turn and with/without ABS?
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "physics/PhysicsWorld.hpp"
#include "vehicle/Electronics.hpp"
#include "vehicle/GroundQuery.hpp"
#include "vehicle/VehicleModel.hpp"

using namespace sim::vehicle;
namespace physics = sim::physics;

namespace {

constexpr Real kDt = 1.0 / 1000.0;
constexpr Real kRadToDeg = 57.295779513082320876;

// A ground defined by an analytic height h(x, z). The normal is the exact
// gradient, so a wheel ray sees the true slope of the surface.
class AnalyticGround : public IGroundQuery {
public:
    explicit AnalyticGround(Real friction = 1.0, Real rollingResistance = 0.015)
        : friction_(friction), rollingResistance_(rollingResistance) {}

    GroundSample sample(Real x, Real z) const override {
        GroundSample result;
        result.height = height(x, z);
        const Real e = 1e-3;
        const Real dx = (height(x + e, z) - height(x - e, z)) / (2.0 * e);
        const Real dz = (height(x, z + e) - height(x, z - e)) / (2.0 * e);
        result.normal = physics::safeNormalize(Vec3(-dx, 1.0, -dz));
        result.friction = friction_;
        result.rollingResistance = rollingResistance_;
        return result;
    }

protected:
    virtual Real height(Real x, Real z) const = 0;

private:
    Real friction_;
    Real rollingResistance_;
};

// A smooth sinusoid in z: gentle crests and troughs the car drives over.
class RippleGround final : public AnalyticGround {
public:
    RippleGround(Real amplitude, Real wavelength, Real friction)
        : AnalyticGround(friction), amplitude_(amplitude), wavelength_(wavelength) {}

protected:
    Real height(Real, Real z) const override { return amplitude_ * std::sin(6.28318530717958647693 * z / wavelength_); }

private:
    Real amplitude_;
    Real wavelength_;
};

// A straight ramp rising as z decreases from 0 to -length (the car drives
// towards -Z), then an abrupt drop back to flat: a launch ramp with a lip.
class RampGround final : public AnalyticGround {
public:
    RampGround(Real slope, Real length) : slope_(slope), length_(length) {}

    Real lipHeight() const { return slope_ * length_; }

protected:
    Real height(Real, Real z) const override {
        if (z >= 0.0) {
            return 0.0;
        }
        if (z <= -length_) {
            return 0.0;
        }
        return slope_ * (-z);
    }

private:
    Real slope_;
    Real length_;
};

// A tiny proportional speed controller so a test can hold a target speed on
// arbitrary terrain.
VehicleInput hold(Real speed, Real target) {
    VehicleInput input;
    input.throttle = std::clamp((target - speed) * 0.5, 0.0, 1.0);
    input.brake = std::clamp((speed - target) * 0.5, 0.0, 1.0);
    return input;
}

// Launches to `entry` m/s on `ground`, then returns the car ready to step.
VehicleModel& launchTo(physics::PhysicsWorld& world, const IGroundQuery& ground, Real entry) {
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    VehicleInput input;
    input.throttle = 1.0;
    car.setInput(input);
    for (int i = 0; i < 90000 && car.forwardSpeed() < entry; ++i) {
        car.step(kDt);
    }
    return car;
}

struct TerrainRun {
    Real contactFraction[4] = {0.0, 0.0, 0.0, 0.0};
    Real minHeightAboveGround = 1e9;
    Real maxHeightAboveGround = -1e9;
    Real chassisYRange = 0.0; // peak-to-peak vertical travel of the body
    Real surfaceRange = 0.0;  // peak-to-peak height of the terrain along the path
    bool finite = true;
};

TerrainRun followTerrain(const IGroundQuery& ground, Real target, Real seconds) {
    physics::PhysicsWorld world;
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    const int steps = static_cast<int>(seconds / kDt);
    TerrainRun run;
    long contact[4] = {0, 0, 0, 0};
    int counted = 0;
    Real minY = 1e9;
    Real maxY = -1e9;
    Real minGround = 1e9;
    Real maxGround = -1e9;
    for (int i = 0; i < steps; ++i) {
        car.setInput(hold(car.forwardSpeed(), target));
        car.step(kDt);
        ++counted;
        if (!car.chassis().isFiniteState()) {
            run.finite = false;
            break;
        }
        for (std::size_t c = 0; c < 4; ++c) {
            if (car.corner(c).contact) {
                ++contact[c];
            }
        }
        const GroundSample surface = ground.sample(car.position().x, car.position().z);
        run.minHeightAboveGround = std::min(run.minHeightAboveGround, car.position().y - surface.height);
        run.maxHeightAboveGround = std::max(run.maxHeightAboveGround, car.position().y - surface.height);
        minY = std::min(minY, car.position().y);
        maxY = std::max(maxY, car.position().y);
        minGround = std::min(minGround, surface.height);
        maxGround = std::max(maxGround, surface.height);
    }
    for (std::size_t c = 0; c < 4; ++c) {
        run.contactFraction[c] = static_cast<Real>(contact[c]) / static_cast<Real>(std::max(1, counted));
    }
    run.chassisYRange = maxY - minY;
    run.surfaceRange = maxGround - minGround;
    return run;
}

struct JumpRun {
    bool airborne = false;
    int airborneSteps = 0;
    Real apexAboveLip = 0.0;
    Real launchSpeed = 0.0;
    bool landed = false;
    bool finite = true;
    Real maxPitchDegrees = 0.0;
};

JumpRun rampJump(const RampGround& ground, int steps) {
    physics::PhysicsWorld world;
    VehicleModel& car = VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
    car.reset(Vec3(0.0, 0.0, 60.0), 0.0);

    JumpRun run;
    const Real lipHeight = ground.lipHeight();
    bool wasAirborne = false;
    VehicleInput input;
    input.throttle = 1.0;
    car.setInput(input);
    for (int i = 0; i < steps; ++i) {
        car.step(kDt);
        if (!car.chassis().isFiniteState()) {
            run.finite = false;
            break;
        }
        const Vec3 up = car.chassis().localToWorldDirection(Vec3(0.0, 1.0, 0.0));
        run.maxPitchDegrees = std::max(run.maxPitchDegrees, std::acos(std::clamp(up.y, -1.0, 1.0)) * kRadToDeg);
        const bool allOff =
            !car.corner(0).contact && !car.corner(1).contact && !car.corner(2).contact && !car.corner(3).contact;
        if (allOff) {
            if (!wasAirborne) {
                run.launchSpeed = car.forwardSpeed();
            }
            run.airborne = true;
            ++run.airborneSteps;
            run.apexAboveLip = std::max(run.apexAboveLip, car.position().y - lipHeight);
        } else if (run.airborne) {
            run.landed = true;
            break;
        }
        wasAirborne = allOff;
    }
    return run;
}

struct BrakeRun {
    Real distance = 0.0;
    int lockedSteps = 0;
    bool absActive = false;
    bool finite = true;
};

BrakeRun hardBrake(const IGroundQuery& ground, Real entry, ElectronicControlUnit* abs) {
    physics::PhysicsWorld world;
    VehicleModel& car = launchTo(world, ground, entry);
    if (abs != nullptr) {
        car.setElectronics(abs);
    }
    const Vec3 start = car.position();
    VehicleInput brake;
    brake.brake = 1.0;
    car.setInput(brake);
    BrakeRun run;
    for (int i = 0; i < 30000 && car.forwardSpeed() > 0.3; ++i) {
        car.step(kDt);
        if (!car.chassis().isFiniteState()) {
            run.finite = false;
            break;
        }
        bool anyLocked = false;
        for (std::size_t c = 0; c < 4; ++c) {
            anyLocked = anyLocked || car.corner(c).spinWheel.locked;
        }
        if (anyLocked) {
            ++run.lockedSteps;
        }
        if (abs != nullptr && abs->absActive()) {
            run.absActive = true;
        }
    }
    const Vec3 d = car.position() - start;
    run.distance = std::sqrt(d.x * d.x + d.z * d.z);
    return run;
}

// Peak body slip angle (rad) over a scripted drift.
Real peakBodySlip(const IGroundQuery& ground, Real entry, Real throttle, Real steer, bool handbrake) {
    physics::PhysicsWorld world;
    VehicleModel& car = launchTo(world, ground, entry);
    VehicleInput drift;
    drift.throttle = throttle;
    drift.steer = steer;
    drift.handbrake = handbrake;
    car.setInput(drift);
    Real maxSlip = 0.0;
    for (int i = 0; i < 2500; ++i) {
        car.step(kDt);
        const Vec3 v = car.linearVelocity();
        const Real lateral = glm::dot(v, car.right());
        maxSlip = std::max(maxSlip, std::abs(std::atan2(lateral, std::max(0.1, std::abs(car.forwardSpeed())))));
    }
    return maxSlip;
}

} // namespace

TEST_CASE("terrain: the wheels follow a smooth undulating surface") {
    const RippleGround ground(0.08, 12.0, 1.0);
    const TerrainRun run = followTerrain(ground, 9.0, 8.0);
    MESSAGE("ripple 0.08/12: contact " << run.contactFraction[0] << ", " << run.contactFraction[1] << ", "
                                       << run.contactFraction[2] << ", " << run.contactFraction[3]
                                       << "; chassis travel " << run.chassisYRange << " m over surface "
                                       << run.surfaceRange << " m");
    REQUIRE(run.finite);
    for (const Real fraction : run.contactFraction) {
        CHECK(fraction > 0.9);
    }
    // All four wheels track the surface and the body rides it without being
    // thrown: its vertical travel stays close to the surface it drives over
    // (a little more, because pitch over the wheelbase adds to it).
    CHECK(run.chassisYRange < run.surfaceRange * 1.5);
    CHECK(run.minHeightAboveGround > 0.15);
}

TEST_CASE("terrain: the wheels cope with a steeper corrugation") {
    const RippleGround ground(0.22, 12.0, 1.0);
    const TerrainRun run = followTerrain(ground, 11.0, 8.0);
    MESSAGE("ripple 0.22/12: contact " << run.contactFraction[0] << ", " << run.contactFraction[1] << ", "
                                       << run.contactFraction[2] << ", " << run.contactFraction[3]
                                       << "; chassis travel " << run.chassisYRange << " m over surface "
                                       << run.surfaceRange << " m");
    REQUIRE(run.finite);
    for (const Real fraction : run.contactFraction) {
        CHECK(fraction > 0.75);
    }
    CHECK(run.minHeightAboveGround > 0.0);
}

TEST_CASE("terrain: a ramp launches the car and it lands cleanly") {
    const RampGround ramp(0.20, 5.0);
    const JumpRun run = rampJump(ramp, 9000);
    MESSAGE("jump: launch " << run.launchSpeed * 3.6 << " km/h, airborne " << run.airborneSteps * kDt << " s, apex "
                            << run.apexAboveLip << " m, max pitch " << run.maxPitchDegrees << " deg");
    REQUIRE(run.finite);
    CHECK(run.airborne);
    CHECK(run.airborneSteps > 50);
    CHECK(run.launchSpeed > 10.0);
    CHECK(run.landed);
    CHECK(run.apexAboveLip > 0.2);
    CHECK(run.apexAboveLip < 4.0);
    CHECK(run.maxPitchDegrees < 80.0);
}

TEST_CASE("dynamics: the car drifts coherently under power") {
    physics::PhysicsWorld world;
    FlatGroundQuery ground(0.0, 0.5); // a slippery surface lets the rear break away
    VehicleModel& car = launchTo(world, ground, 14.0);
    const Real heading0 = car.heading();
    VehicleInput drift;
    drift.throttle = 0.8;
    drift.steer = 0.8;
    car.setInput(drift);
    Real maxSlip = 0.0;
    Real maxRearSlipAngle = 0.0;
    Real maxYaw = 0.0;
    for (int i = 0; i < 2500; ++i) {
        car.step(kDt);
        REQUIRE(car.chassis().isFiniteState());
        const Vec3 v = car.linearVelocity();
        const Real lateral = glm::dot(v, car.right());
        maxSlip = std::max(maxSlip, std::abs(std::atan2(lateral, std::max(0.1, std::abs(car.forwardSpeed())))));
        maxRearSlipAngle = std::max(maxRearSlipAngle, std::abs(car.corner(2).slipAngle));
        maxYaw = std::max(maxYaw, car.angularVelocity().y);
    }
    MESSAGE("drift: body slip " << maxSlip * kRadToDeg << " deg, rear slip " << maxRearSlipAngle * kRadToDeg
                                << " deg, max yaw " << maxYaw << " rad/s, heading delta "
                                << (car.heading() - heading0) * kRadToDeg << " deg");
    CHECK(maxSlip > 0.05);          // the car really slides
    CHECK(maxSlip < 1.2);           // but keeps it together (no spin past ~70 deg)
    CHECK(maxRearSlipAngle > 0.08); // the rear tyres work past their peak slip
    CHECK(maxYaw > 0.0);            // consistent with the left steer
    CHECK((car.heading() - heading0) > 0.0);
}

TEST_CASE("dynamics: the handbrake drifts the car further than throttle alone") {
    FlatGroundQuery ground(0.0, 0.6);
    // A committed handbrake turn: enough speed and steering lock that the locked
    // rear wheels break away, against the same corner taken on throttle alone.
    const Real plain = peakBodySlip(ground, 20.0, 0.3, 1.0, false);
    const Real pulled = peakBodySlip(ground, 20.0, 0.3, 1.0, true);
    MESSAGE("slip: throttle only " << plain * kRadToDeg << " deg, with handbrake " << pulled * kRadToDeg << " deg");
    CHECK(pulled > plain);
    CHECK(pulled > 0.5); // a real drift: the handbrake rotates the car well past the throttle-only line
}

TEST_CASE("dynamics: braking in a straight line stays straight") {
    physics::PhysicsWorld world;
    FlatGroundQuery ground(0.0, 1.0);
    VehicleModel& car = launchTo(world, ground, 25.0);
    const Real heading0 = car.heading();
    const Vec3 start = car.position();
    VehicleInput brake;
    brake.brake = 1.0;
    car.setInput(brake);
    Real maxLateral = 0.0;
    for (int i = 0; i < 30000 && car.forwardSpeed() > 0.3; ++i) {
        car.step(kDt);
        REQUIRE(car.chassis().isFiniteState());
        const Vec3 v = car.linearVelocity();
        maxLateral = std::max(maxLateral, std::abs(glm::dot(v, car.right())));
    }
    const Vec3 d = car.position() - start;
    MESSAGE("straight braking: distance " << std::sqrt(d.x * d.x + d.z * d.z) << " m, max |lateral velocity| "
                                          << maxLateral << " m/s");
    CHECK(std::abs(car.heading() - heading0) < 0.05);
    CHECK(car.forwardSpeed() < 0.5);
    CHECK(maxLateral < 0.5);
}

TEST_CASE("dynamics: braking into a turn slows the car without spinning it") {
    physics::PhysicsWorld world;
    FlatGroundQuery ground(0.0, 1.0);
    VehicleModel& car = launchTo(world, ground, 18.0);
    const Real heading0 = car.heading();
    VehicleInput corner;
    corner.steer = 0.5;
    corner.throttle = 0.3;
    car.setInput(corner);
    for (int i = 0; i < 800; ++i) {
        car.step(kDt);
    }
    VehicleInput brake;
    brake.steer = 0.5;
    brake.brake = 1.0;
    car.setInput(brake);
    for (int i = 0; i < 6000 && car.forwardSpeed() > 0.5; ++i) {
        car.step(kDt);
        REQUIRE(car.chassis().isFiniteState());
    }
    const Real headingDelta = std::abs(car.heading() - heading0);
    MESSAGE("brake-in-turn: final speed " << car.forwardSpeed() << " m/s, heading delta " << headingDelta * kRadToDeg
                                          << " deg");
    CHECK(car.forwardSpeed() < 3.0);
    CHECK(headingDelta < 2.0); // turned, but did not spin out
}

TEST_CASE("dynamics: ABS keeps the wheels from locking and shortens the stop") {
    const FlatGroundQuery ground(0.0, 0.4);
    const BrakeRun skid = hardBrake(ground, 22.0, nullptr);

    ElectronicControlUnit abs;
    abs.setEnabled(true, false, false);
    const BrakeRun aided = hardBrake(ground, 22.0, &abs);

    MESSAGE("low grip 0.4 from 22 m/s: no ABS " << skid.distance << " m (" << skid.lockedSteps << " locked steps), ABS "
                                                << aided.distance << " m (" << aided.lockedSteps
                                                << " locked steps, active " << aided.absActive << ")");
    REQUIRE(skid.finite);
    REQUIRE(aided.finite);
    CHECK(skid.lockedSteps > 0);                 // the unaided car locks a wheel
    CHECK(aided.lockedSteps < skid.lockedSteps); // ABS releases it
    CHECK(aided.absActive);
    CHECK(aided.distance < skid.distance * 0.9); // ABS shortens the stop, not just unlocks the wheel
}
