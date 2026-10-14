// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <cmath>
#include <sstream>

// Every interface header is included so that it is compile-checked here.
#include "vehicle/IAerodynamics.hpp"
#include "vehicle/IBrakeSystem.hpp"
#include "vehicle/IDrivetrain.hpp"
#include "vehicle/ISuspension.hpp"
#include "vehicle/ITireModel.hpp"
#include "vehicle/IVehicleElectronics.hpp"
#include "vehicle/IVehicleModel.hpp"
#include "vehicle/MagicFormulaTire.hpp"
#include "vehicle/PlaceholderVehicle.hpp"
#include "vehicle/TireVertical.hpp"

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

TEST_CASE("MagicFormulaTire matches the pure-slip Magic Formula curve") {
    MagicFormulaTire tire;
    const MagicFormulaCoefficients& c = tire.coefficients();
    const Real fz0 = c.nominalLoad;

    const auto state = [&](Real kappa, Real alpha) {
        TireState s;
        s.slipRatio = kappa;
        s.slipAngle = alpha;
        s.normalLoad = fz0;
        return s;
    };

    SUBCASE("zero slip gives zero force and a lifted tyre too") {
        const TireForces rest = tire.computeForces(state(0.0, 0.0));
        CHECK(rest.longitudinal == doctest::Approx(0.0));
        CHECK(rest.lateral == doctest::Approx(0.0));

        TireState lifted = state(0.2, 0.1);
        lifted.normalLoad = 0.0;
        const TireForces none = tire.computeForces(lifted);
        CHECK(none.longitudinal == doctest::Approx(0.0));
        CHECK(none.lateral == doctest::Approx(0.0));
    }

    SUBCASE("slope at the origin equals the slip stiffness BCD = K") {
        const Real eps = 1e-5;
        const Real fx = tire.computeForces(state(eps, 0.0)).longitudinal;
        const Real kx = fz0 * (c.pKx1 + c.pKx2 * 0.0) * std::exp(c.pKx3 * 0.0);
        CHECK(fx / eps == doctest::Approx(kx).epsilon(1e-3));

        const Real fy = tire.computeForces(state(0.0, eps)).lateral;
        const Real ky = c.pKy1 * fz0 * std::sin(2.0 * std::atan(fz0 / (c.pKy2 * fz0)));
        CHECK(-fy / eps == doctest::Approx(ky).epsilon(1e-3));
        MESSAGE("tire origin slopes: Kx = " << kx << " N, Ky = " << ky << " N/rad");
    }

    SUBCASE("the peak force equals mu times the load") {
        Real fxPeak = 0.0;
        Real fyPeak = 0.0;
        for (int i = 0; i <= 2000; ++i) {
            const Real x = 0.5 * static_cast<Real>(i) / 2000.0;
            fxPeak = std::max(fxPeak, tire.computeForces(state(x, 0.0)).longitudinal);
            fyPeak = std::max(fyPeak, std::abs(tire.computeForces(state(0.0, x)).lateral));
        }
        CHECK(fxPeak == doctest::Approx(c.pDx1 * fz0).epsilon(0.01));
        CHECK(fyPeak == doctest::Approx(c.pDy1 * fz0).epsilon(0.01));
        MESSAGE("tire peak force at Fz0=4000 N: Fx = " << fxPeak << " N, Fy = " << fyPeak << " N");
    }

    SUBCASE("the force is odd in the slip and opposes the slip angle") {
        const Real a = tire.computeForces(state(0.15, 0.0)).longitudinal;
        const Real b = tire.computeForces(state(-0.15, 0.0)).longitudinal;
        CHECK(a == doctest::Approx(-b));

        const Real left = tire.computeForces(state(0.0, 0.1)).lateral;
        const Real right = tire.computeForces(state(0.0, -0.1)).lateral;
        CHECK(left < 0.0);
        CHECK(right > 0.0);
        CHECK(left == doctest::Approx(-right));
    }

    SUBCASE("the peak grip falls with load (load sensitivity)") {
        const auto peakAt = [&](Real load) {
            Real peak = 0.0;
            for (int i = 0; i <= 2000; ++i) {
                TireState s = state(0.5 * static_cast<Real>(i) / 2000.0, 0.0);
                s.normalLoad = load;
                peak = std::max(peak, tire.computeForces(s).longitudinal);
            }
            return peak;
        };
        const Real light = peakAt(fz0);
        const Real heavy = peakAt(2.0 * fz0);
        CHECK(heavy > light);       // more load still grips more
        CHECK(heavy < 2.0 * light); // but less than proportionally
    }

    SUBCASE("combined slip stays inside the friction circle") {
        TireState s = state(0.18, 0.14);
        const TireForces f = tire.computeForces(s);
        const Real magnitude = std::sqrt(f.longitudinal * f.longitudinal + f.lateral * f.lateral);
        CHECK(magnitude <= c.pDx1 * fz0 * 1.0000001);
    }

    SUBCASE("self-aligning moment follows the pneumatic trail") {
        const Real eps = 1e-5;
        const Real ky = c.pKy1 * fz0 * std::sin(2.0 * std::atan(fz0 / (c.pKy2 * fz0)));
        const Real mz = tire.computeForces(state(0.0, eps)).aligningMoment;
        CHECK(mz / eps == doctest::Approx(c.qDz1 * ky).epsilon(1e-2));

        // Mz is odd in the slip angle and vanishes at zero slip.
        CHECK(tire.computeForces(state(0.0, 0.0)).aligningMoment == doctest::Approx(0.0));
        const Real left = tire.computeForces(state(0.0, 0.1)).aligningMoment;
        const Real right = tire.computeForces(state(0.0, -0.1)).aligningMoment;
        CHECK(left == doctest::Approx(-right));

        // The trail falls as the tyre saturates: |Mz| peaks then decreases.
        Real best = 0.0;
        int bestIndex = 0;
        constexpr int kSamples = 400;
        for (int i = 1; i <= kSamples; ++i) {
            const Real alpha = 0.8 * static_cast<Real>(i) / kSamples;
            const Real magnitude = std::abs(tire.computeForces(state(0.0, alpha)).aligningMoment);
            if (magnitude > best) {
                best = magnitude;
                bestIndex = i;
            }
        }
        MESSAGE("peak self-aligning moment = " << best << " N*m at alpha sample " << bestIndex);
        CHECK(bestIndex > 0);
        CHECK(bestIndex < kSamples);
        CHECK(best > 0.0);
    }

    CHECK(tire.unloadedRadius() == doctest::Approx(c.radius));
}

TEST_CASE("MF tyre camber thrust and rolling resistance") {
    MagicFormulaTire tire;
    const MagicFormulaCoefficients& c = tire.coefficients();
    const Real fz0 = c.nominalLoad;

    TireState leaned;
    leaned.normalLoad = fz0;
    leaned.camber = 0.05;
    const TireForces camberForces = tire.computeForces(leaned);
    CHECK(camberForces.lateral == doctest::Approx(c.pKyg1 * fz0 * 0.05));

    // Reversed camber flips the thrust.
    TireState other = leaned;
    other.camber = -0.05;
    CHECK(tire.computeForces(other).lateral == doctest::Approx(-camberForces.lateral));

    // Rolling resistance opposes travel and vanishes at rest.
    TireState rolling = leaned;
    rolling.camber = 0.0;
    rolling.contactSpeed = 10.0;
    const TireForces forward = tire.computeForces(rolling);
    CHECK(forward.rollingResistanceMoment == doctest::Approx(-c.rollingResistance * fz0 * c.radius));

    TireState backward = rolling;
    backward.contactSpeed = -10.0;
    CHECK(tire.computeForces(backward).rollingResistanceMoment == doctest::Approx(-forward.rollingResistanceMoment));
    CHECK(tire.computeForces(leaned).rollingResistanceMoment == doctest::Approx(0.0)); // contactSpeed = 0
}

TEST_CASE("the vertical tyre never pulls and its radius follows the load") {
    TireVerticalCoefficients c;

    // Airborne: no deflection, so no load and the full unloaded radius.
    TireVerticalState airborne;
    airborne.deflection = -0.01; // centre above the road
    const TireVerticalResult up = evaluateTireVertical(c, airborne);
    CHECK(up.normalLoad == doctest::Approx(0.0));
    CHECK(up.effectiveRadius == doctest::Approx(c.unloadedRadius));
    CHECK(up.contactHalfLength == doctest::Approx(0.0));

    // Touching but receding: the one-sided spring must not pull.
    TireVerticalState receding;
    receding.deflection = 0.0;
    receding.deflectionRate = -5.0;
    CHECK(evaluateTireVertical(c, receding).normalLoad == doctest::Approx(0.0));

    // Loaded: the load rises with deflection and the damping adds to it.
    TireVerticalState soft;
    soft.deflection = 0.01;
    TireVerticalState hard;
    hard.deflection = 0.02;
    const TireVerticalResult softR = evaluateTireVertical(c, soft);
    const TireVerticalResult hardR = evaluateTireVertical(c, hard);
    CHECK(softR.normalLoad > 0.0);
    CHECK(hardR.normalLoad > softR.normalLoad);
    CHECK(softR.loadedRadius == doctest::Approx(c.unloadedRadius - 0.01));
    CHECK(softR.contactHalfLength == doctest::Approx(std::sqrt(0.31 * 0.31 - 0.30 * 0.30)));

    TireVerticalState compress = soft;
    compress.deflectionRate = 1.0;
    CHECK(evaluateTireVertical(c, compress).normalLoad > softR.normalLoad);

    // Effective radius shrinks with load and grows with spin.
    CHECK(softR.effectiveRadius < c.unloadedRadius);
    TireVerticalState spinning = soft;
    c.radiusOmega = 0.01;
    spinning.spin = 30.0;
    CHECK(evaluateTireVertical(c, spinning).effectiveRadius > softR.effectiveRadius);
}

TEST_CASE("a .tir file parses into the pure-slip coefficients") {
    std::istringstream tir("! TNO MF-Tyre\n"
                           "[MODEL]\n"
                           "FNOMIN = 4000\n"
                           "UNLOADED_RADIUS = 0.31\n"
                           "[COEFFICIENTS]\n"
                           "PCX1 = 1.8   $ longitudinal shape\n"
                           "PDX1 = 1.5\n"
                           "PKX1 = 26\n"
                           "PKY1 = 14\n"
                           "PKY2 = 2.2\n"
                           "SOME_OTHER_COEFFICIENT = 99\n"
                           "[VERTICAL]\n"
                           "VERTICAL_STIFFNESS = 200000\n");

    const MagicFormulaCoefficients c = parseMagicFormulaCoefficients(tir);
    CHECK(c.nominalLoad == doctest::Approx(4000.0));
    CHECK(c.radius == doctest::Approx(0.31));
    CHECK(c.pCx1 == doctest::Approx(1.8));
    CHECK(c.pDx1 == doctest::Approx(1.5));
    CHECK(c.pKx1 == doctest::Approx(26.0));
    CHECK(c.pKy1 == doctest::Approx(14.0));
    CHECK(c.pKy2 == doctest::Approx(2.2));
    CHECK(c.pDx2 == doctest::Approx(MagicFormulaCoefficients{}.pDx2)); // untouched default

    // A parse followed by a model evaluation works end to end.
    MagicFormulaTire tire(c);
    TireState state;
    state.normalLoad = c.nominalLoad;
    state.slipRatio = 0.1;
    CHECK(tire.computeForces(state).longitudinal > 0.0);
}
