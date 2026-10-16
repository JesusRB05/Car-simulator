// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <array>
#include <cmath>

#include "physics/PhysicsWorld.hpp"
#include "vehicle/GroundQuery.hpp"
#include "vehicle/MagicFormulaTire.hpp"
#include "vehicle/VehicleModel.hpp"

using namespace sim::vehicle;
namespace physics = sim::physics;

namespace {

constexpr Real kDt = 1.0 / 1000.0;
constexpr Real kG = 9.80665;

// Reference friction the car's own tyre can still develop, instead of hard-coded
// tolerances. A fully braked wheel locks and slides at slip ratio -1, so the
// deceleration ceiling without ABS is mu_x(-1) g; the friction bound caps it at
// friction * mu_peak on a low-grip surface. Both come from the tyre parameters.
Real lockedWheelMu() {
    const MagicFormulaCoefficients coefficients = makeDefaultVehicleSetup().tyre;
    MagicFormulaTire tyre(coefficients);
    TireState state;
    state.slipRatio = -1.0;
    state.normalLoad = coefficients.nominalLoad;
    state.frictionCoefficient = 1.0;
    return -tyre.computeForces(state).longitudinal / state.normalLoad;
}

Real peakLateralMu() {
    return makeDefaultVehicleSetup().tyre.pDy1;
}

// The surface friction scales the whole tyre curve, so the locked-wheel friction
// on a surface of `friction` is just `friction` times the dry value.
Real gripLimitedMu(Real friction) {
    return friction * lockedWheelMu();
}

// Reference stopping distance: v^2 / (2 mu g) for the grip-limited deceleration.
Real referenceStop(Real friction, Real entry) {
    return entry * entry / (2.0 * gripLimitedMu(friction) * kG);
}

// A car on an infinite flat asphalt plane of a given grip. The ground outlives
// the model because it is a member declared before the model is spawned.
struct Machine {
    physics::PhysicsWorld world;
    FlatGroundQuery ground;
    VehicleModel* car = nullptr;
    Vec3 origin{0.0};

    explicit Machine(Real friction) : ground(0.0, friction) {
        car = &VehicleModel::spawn(world, makeDefaultVehicleSetup(), ground);
        origin = car->position();
    }

    Real advance() {
        car->step(kDt);
        return car->forwardSpeed();
    }
};

// Time (s) to reach `target` m/s under full throttle, or -1 if it never does.
Real timeToSpeed(Real friction, Real target, int maxSteps = 90000) {
    Machine m(friction);
    VehicleInput input;
    input.throttle = 1.0;
    m.car->setInput(input);
    for (int i = 0; i < maxSteps; ++i) {
        if (m.advance() >= target) {
            return i * kDt;
        }
    }
    return -1.0;
}

// Distance (m) to go from `entry` m/s to below 0.5 m/s under full braking.
Real stoppingDistance(Real friction, Real entry, int maxSteps = 30000) {
    Machine m(friction);
    VehicleInput input;
    input.throttle = 1.0;
    m.car->setInput(input);
    for (int i = 0; i < 90000 && m.car->forwardSpeed() < entry; ++i) {
        m.advance();
    }
    const Vec3 start = m.car->position();
    VehicleInput brakeInput;
    brakeInput.brake = 1.0;
    m.car->setInput(brakeInput);
    for (int i = 0; i < maxSteps && m.car->forwardSpeed() > 0.5; ++i) {
        m.advance();
    }
    const Vec3 d = m.car->position() - start;
    return std::sqrt(d.x * d.x + d.z * d.z);
}

// Top speed (m/s) reached under full throttle in `seconds`.
Real topSpeed(Real friction, Real seconds) {
    Machine m(friction);
    VehicleInput input;
    input.throttle = 1.0;
    m.car->setInput(input);
    Real best = 0.0;
    const int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        best = std::max(best, m.advance());
    }
    return best;
}

// Quarter mile time and trap speed.
struct Quarter {
    Real time = -1.0;
    Real trap = 0.0;
};

Quarter quarterMile(Real friction) {
    Machine m(friction);
    VehicleInput input;
    input.throttle = 1.0;
    m.car->setInput(input);
    Quarter result;
    for (int i = 0; i < 90000; ++i) {
        const Real speed = m.advance();
        const Vec3 d = m.car->position() - m.origin;
        if (std::sqrt(d.x * d.x + d.z * d.z) >= 402.336) {
            result.time = i * kDt;
            result.trap = speed;
            break;
        }
    }
    return result;
}

// Steady-state cornering: settle at a fixed steer/throttle, then report the mean
// lateral acceleration (m/s^2) and speed.
struct Corner {
    Real speed = 0.0;
    Real lateralG = 0.0;
    Real radius = 0.0;
};

Corner skidpad(Real friction, Real entrySpeed, Real steer, Real throttle) {
    Machine m(friction);
    VehicleInput launch;
    launch.throttle = 1.0;
    m.car->setInput(launch);
    for (int i = 0; i < 90000 && m.car->forwardSpeed() < entrySpeed; ++i) {
        m.advance();
    }
    VehicleInput input;
    input.steer = steer;
    input.throttle = throttle;
    m.car->setInput(input);
    for (int i = 0; i < 4000; ++i) {
        m.advance();
    }
    Real speedSum = 0.0;
    Real yawSum = 0.0;
    const int samples = 2000;
    for (int i = 0; i < samples; ++i) {
        m.advance();
        speedSum += m.car->forwardSpeed();
        yawSum += std::abs(m.car->angularVelocity().y);
    }
    Corner corner;
    corner.speed = speedSum / samples;
    const Real yaw = yawSum / samples;
    corner.lateralG = corner.speed * yaw / kG;
    corner.radius = yaw > 1e-9 ? corner.speed / yaw : 1e9;
    return corner;
}

} // namespace

TEST_CASE("performance: 0-100 km/h across a grip sweep is power- and grip-limited") {
    constexpr std::array<Real, 8> grips{1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.35};
    const Real target = 100.0 / 3.6;
    Real first = 0.0;
    Real previous = -1.0;
    for (Real grip : grips) {
        const Real time = timeToSpeed(grip, target);
        MESSAGE("grip " << grip << " -> 0-100 km/h " << time << " s");
        REQUIRE(time > 0.0);
        CHECK(time > 2.0);
        CHECK(time < 20.0);
        if (previous < 0.0) {
            first = time;
        } else {
            // A whole tyre step of slip rarely moves the launch by more than a
            // couple of tenths of a second in either direction.
            CHECK(time >= previous - 0.5);
        }
        previous = time;
    }
    // The lowest grip is clearly no quicker than the highest.
    CHECK(previous >= first - 0.5);
}

TEST_CASE("performance: 0-50 and 0-160 km/h bracket the 0-100 result") {
    const Real to50 = timeToSpeed(1.0, 50.0 / 3.6);
    const Real to100 = timeToSpeed(1.0, 100.0 / 3.6);
    const Real to160 = timeToSpeed(1.0, 160.0 / 3.6);
    const Real lowGrip = timeToSpeed(0.35, 100.0 / 3.6);
    MESSAGE("0-50 " << to50 << " s, 0-100 " << to100 << " s, 0-160 km/h " << to160 << " s");
    CHECK(to50 > 1.0);
    CHECK(to50 < 6.0);
    CHECK(to160 > to100);
    CHECK(to160 < 25.0);
    // Lower grip slows the launch.
    CHECK(lowGrip > to100);
}

TEST_CASE("performance: top speed is finite and drag limited") {
    const Real high = topSpeed(1.0, 40.0);
    const Real low = topSpeed(0.6, 40.0);
    MESSAGE("top speed: grip 1.0 -> " << high * 3.6 << " km/h, grip 0.6 -> " << low * 3.6 << " km/h");
    CHECK(std::isfinite(high));
    CHECK(high > 45.0); // above 162 km/h
    CHECK(high < 90.0); // and below 324 km/h
    CHECK(low < high + 1.0);
}

TEST_CASE("performance: the quarter mile is a plausible street-car number") {
    const Quarter high = quarterMile(1.0);
    const Quarter low = quarterMile(0.4);
    MESSAGE("quarter mile: grip 1.0 -> " << high.time << " s @ " << high.trap * 3.6 << " km/h, grip 0.4 -> " << low.time
                                         << " s @ " << low.trap * 3.6 << " km/h");
    CHECK(high.time > 8.0);
    CHECK(high.time < 25.0);
    CHECK(high.trap > 15.0);
    CHECK(high.trap < 60.0);
    CHECK(low.time >= high.time);
}

TEST_CASE("performance: braking distance grows with entry speed (v^2 law)") {
    const Real reference = lockedWheelMu() * kG;
    constexpr std::array<Real, 6> speeds{10.0, 15.0, 20.0, 25.0, 30.0, 35.0};
    Real previous = 0.0;
    for (const Real speed : speeds) {
        const Real distance = stoppingDistance(1.0, speed);
        const Real deceleration = speed * speed / (2.0 * distance);
        MESSAGE("brake " << speed << " m/s -> " << distance << " m (a = " << deceleration << " m/s^2, ref " << reference
                         << ")");
        CHECK(distance > 0.0);
        CHECK(distance > previous);
        // A dry stop is grip-limited, so the mean deceleration is the tyre's
        // locked-wheel friction times g, within a 15% band (the small spread with
        // speed is the aerodynamic drag and the brake-application transient).
        CHECK(deceleration > reference * 0.85);
        CHECK(deceleration < reference * 1.15);
        previous = distance;
    }
}

TEST_CASE("performance: a dry stop is grip-limited and the distance scales with grip") {
    const Real dry = stoppingDistance(1.0, 25.0);
    const Real damp = stoppingDistance(0.6, 25.0);
    const Real slippery = stoppingDistance(0.4, 25.0);
    const Real ice = stoppingDistance(0.3, 25.0);
    MESSAGE("brake from 25 m/s: grip 1.0 -> " << dry << " m (ref " << referenceStop(1.0, 25.0) << "), 0.6 -> " << damp
                                              << " m, 0.4 -> " << slippery << " m, 0.3 -> " << ice << " m");
    // The brakes are sized to lock the wheels on dry asphalt, so the dry stop is
    // limited by the tyre, not the brakes: it lands within 15% of the reference
    // v^2 / (2 mu g).
    const auto withinReference = [](Real distance, Real reference) {
        return distance > reference * 0.85 && distance < reference * 1.15;
    };
    CHECK(withinReference(dry, referenceStop(1.0, 25.0)));
    CHECK(withinReference(damp, referenceStop(0.6, 25.0)));
    CHECK(withinReference(slippery, referenceStop(0.4, 25.0)));
    CHECK(withinReference(ice, referenceStop(0.3, 25.0)));
    // And the distance climbs monotonically as the surface gets more slippery.
    CHECK(damp > dry);
    CHECK(slippery > damp);
    CHECK(ice > slippery);
}

TEST_CASE("performance: steady-state cornering reaches a realistic lateral acceleration") {
    const Corner low = skidpad(1.0, 8.0, 0.6, 0.2);
    const Corner high = skidpad(1.0, 18.0, 0.35, 0.35);
    const Real reference = peakLateralMu(); // peak lateral grip from the tyre, in g
    MESSAGE("cornering: low " << low.speed << " m/s, " << low.lateralG << " g, R " << low.radius << " m | high "
                              << high.speed << " m/s, " << high.lateralG << " g, R " << high.radius << " m (peak "
                              << reference << " g)");
    // The high-speed steady turn runs close to the tyre's lateral limit; the
    // partial throttle spends part of the friction circle on drive, so it lands
    // between 70% and 100% of the peak lateral friction from the tyre parameters.
    CHECK(high.lateralG > reference * 0.70);
    CHECK(high.lateralG < reference * 1.00);
    // The tight, slow turn develops real lateral load but stays well under it.
    CHECK(low.lateralG > reference * 0.30);
    CHECK(low.lateralG < high.lateralG);
    CHECK(high.speed > low.speed);
}
