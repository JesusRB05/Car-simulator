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

TEST_CASE("combined slip stays inside the friction circle in both forms") {
    MagicFormulaTire tire;
    const Real fz0 = tire.coefficients().nominalLoad;
    const Real limit = std::max(tire.coefficients().pDx1, tire.coefficients().pDy1) * fz0;

    for (const CombinedSlipModel model : {CombinedSlipModel::WeightingFunctions, CombinedSlipModel::SlipVector}) {
        tire.setCombinedSlipModel(model);
        for (int ki = -10; ki <= 10; ++ki) {
            for (int ai = -6; ai <= 6; ++ai) {
                TireState s;
                s.normalLoad = fz0;
                s.slipRatio = 0.1 * static_cast<Real>(ki);
                s.slipAngle = 0.1 * static_cast<Real>(ai);
                const TireForces f = tire.computeForces(s);
                const Real magnitude = std::sqrt(f.longitudinal * f.longitudinal + f.lateral * f.lateral);
                CHECK(magnitude <= limit * 1.0000001);
                // Zero slip gives zero force in both formulations.
                if (ki == 0 && ai == 0) {
                    CHECK(magnitude == doctest::Approx(0.0));
                }
            }
        }
    }
}

TEST_CASE("the weighting functions never exceed the pure-slip force") {
    MagicFormulaTire tire; // weighting functions by default
    const Real fz0 = tire.coefficients().nominalLoad;

    const auto state = [&](Real kappa, Real alpha) {
        TireState s;
        s.normalLoad = fz0;
        s.slipRatio = kappa;
        s.slipAngle = alpha;
        return s;
    };

    for (int i = 1; i <= 10; ++i) {
        const Real kappa = 0.05 * static_cast<Real>(i);
        const Real alpha = 0.05 * static_cast<Real>(i);

        const TireForces pureX = tire.computeForces(state(kappa, 0.0));
        const TireForces combX = tire.computeForces(state(kappa, alpha));
        CHECK(combX.longitudinal > 0.0);                                     // still driving
        CHECK(std::abs(combX.longitudinal) <= std::abs(pureX.longitudinal)); // and reduced

        const TireForces pureY = tire.computeForces(state(0.0, alpha));
        const TireForces combY = tire.computeForces(state(kappa, alpha));
        CHECK(combY.lateral < 0.0);                                // still opposes alpha
        CHECK(std::abs(combY.lateral) <= std::abs(pureY.lateral)); // and reduced
    }
}

TEST_CASE("the slip-vector form aligns the resultant with the slip direction") {
    MagicFormulaTire tire;
    tire.setCombinedSlipModel(CombinedSlipModel::SlipVector);
    const MagicFormulaCoefficients& c = tire.coefficients();
    const Real fz0 = c.nominalLoad;

    const auto state = [&](Real kappa, Real alpha) {
        TireState s;
        s.normalLoad = fz0;
        s.slipRatio = kappa;
        s.slipAngle = alpha;
        return s;
    };

    // The resultant peaks (equals mu * Fz) exactly at the reference slip.
    const TireForces peak = tire.computeForces(state(c.peakSlipRatio, 0.0));
    CHECK(peak.longitudinal == doctest::Approx(c.pDx1 * fz0).epsilon(1e-6));

    // Sign follows the slip: driving is +Fx, a positive slip angle gives -Fy.
    const TireForces f = tire.computeForces(state(0.2, 0.15));
    CHECK(f.longitudinal > 0.0);
    CHECK(f.lateral < 0.0);
    const TireForces mirrored = tire.computeForces(state(-0.2, -0.15));
    CHECK(mirrored.longitudinal == doctest::Approx(-f.longitudinal));
    CHECK(mirrored.lateral == doctest::Approx(-f.lateral));
}

TEST_CASE("the residual, overturning and rolling-resistance moments follow their coefficients") {
    const MagicFormulaCoefficients c;
    MagicFormulaTire tire(c);
    const Real fz0 = c.nominalLoad;

    const auto state = [&](Real alpha, Real v) {
        TireState s;
        s.normalLoad = fz0;
        s.slipAngle = alpha;
        s.contactSpeed = v;
        return s;
    };

    SUBCASE("rolling resistance grows with speed") {
        const TireForces slow = tire.computeForces(state(0.0, 0.0));
        CHECK(slow.rollingResistanceMoment == doctest::Approx(-c.rollingResistanceC0 * fz0 * c.radius));

        MagicFormulaCoefficients faster = c;
        faster.rollingResistanceC1 = 0.002;
        MagicFormulaTire fastTire(faster);
        const TireForces fast = fastTire.computeForces(state(0.0, 30.0));
        CHECK(std::abs(fast.rollingResistanceMoment) > std::abs(slow.rollingResistanceMoment));
    }

    SUBCASE("overturning moment follows camber and lateral force") {
        MagicFormulaCoefficients oc = c;
        oc.qSx1 = 0.02;
        oc.qSx2 = 0.05;
        oc.qSx3 = 0.1;
        MagicFormulaTire overTire(oc);

        TireState s = state(0.0, 0.0);
        CHECK(overTire.computeForces(s).overturningMoment == doctest::Approx(c.radius * fz0 * 0.02));

        s.camber = 0.1;
        CHECK(overTire.computeForces(s).overturningMoment == doctest::Approx(c.radius * fz0 * (0.02 - 0.05 * 0.1)));

        s.camber = 0.0;
        s.slipAngle = 0.1;
        const TireForces f = overTire.computeForces(s);
        CHECK(f.overturningMoment == doctest::Approx(c.radius * fz0 * (0.02 + 0.1 * f.lateral / fz0)));
    }

    SUBCASE("the residual torque is off by default and even in the slip angle") {
        CHECK(tire.computeForces(state(0.0, 0.0)).aligningMoment == doctest::Approx(0.0));

        MagicFormulaCoefficients rc = c;
        rc.qDr1 = 0.02;
        MagicFormulaTire resTire(rc);
        const Real expected = c.radius * fz0 * 0.02;
        CHECK(resTire.computeForces(state(0.0, 0.0)).aligningMoment == doctest::Approx(expected));

        // The trail torque is odd in alpha, so the mean of the two signs is the
        // even residual torque.
        const Real a = 0.05;
        const Real plus = resTire.computeForces(state(a, 0.0)).aligningMoment;
        const Real minus = resTire.computeForces(state(-a, 0.0)).aligningMoment;
        CHECK(0.5 * (plus + minus) == doctest::Approx(expected).epsilon(0.02));
    }
}

TEST_CASE("the tyre relaxation filters the slip over a rolled distance") {
    const MagicFormulaCoefficients c;
    const Real speed = 20.0;
    const Real dt = 1.0 / 1000.0;
    const Real tauLateral = c.relaxationLengthLateral / speed;
    const int stepsToTau = static_cast<int>(std::lround(tauLateral / dt));

    TireState raw;
    raw.normalLoad = c.nominalLoad;
    raw.contactSpeed = speed;
    raw.slipAngle = 0.1;
    raw.slipRatio = 0.1;

    SUBCASE("a step in the slip angle reaches 1 - 1/e at one time constant") {
        TireRelaxation rel;
        for (int i = 0; i < stepsToTau; ++i) {
            rel.update(c, raw, dt);
        }
        CHECK(rel.slipAngle / raw.slipAngle == doctest::Approx(1.0 - std::exp(-1.0)).epsilon(1e-6));

        for (int i = 0; i < 10 * stepsToTau; ++i) {
            rel.update(c, raw, dt);
        }
        CHECK(rel.slipAngle == doctest::Approx(raw.slipAngle).epsilon(1e-3));
    }

    SUBCASE("a step in the slip ratio uses the longitudinal length") {
        TireRelaxation rel;
        const int steps = static_cast<int>(std::lround((c.relaxationLengthLongitudinal / speed) / dt));
        for (int i = 0; i < steps; ++i) {
            rel.update(c, raw, dt);
        }
        CHECK(rel.slipRatio / raw.slipRatio == doctest::Approx(1.0 - std::exp(-1.0)).epsilon(1e-6));
    }

    SUBCASE("a slower speed relaxes more slowly and the load is carried through") {
        TireRelaxation fast;
        TireRelaxation slow;
        TireState crawl = raw;
        crawl.contactSpeed = 1.0;
        for (int i = 0; i < stepsToTau; ++i) {
            fast.update(c, raw, dt);
            slow.update(c, crawl, dt);
        }
        CHECK(fast.slipAngle > slow.slipAngle);

        const TireState out = slow.update(c, crawl, dt);
        CHECK(out.slipAngle == doctest::Approx(slow.slipAngle));
        CHECK(out.normalLoad == doctest::Approx(c.nominalLoad));
    }
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
                           "RBY1 = 9.5\n"
                           "RBY2 = 7.5\n"
                           "RCY1 = 1.2\n"
                           "RHY1 = 0.01\n"
                           "QSX1 = 0.02\n"
                           "QDR1 = 0.015\n"
                           "ROLLING_RESISTANCE = 0.012\n"
                           "SIGMA_X = 0.35\n"
                           "SIGMA_Y = 0.45\n"
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
    CHECK(c.rBy1 == doctest::Approx(9.5));
    CHECK(c.rBy2 == doctest::Approx(7.5));
    CHECK(c.rCy1 == doctest::Approx(1.2));
    CHECK(c.rHy1 == doctest::Approx(0.01));
    CHECK(c.qSx1 == doctest::Approx(0.02));
    CHECK(c.qDr1 == doctest::Approx(0.015));
    CHECK(c.rollingResistanceC0 == doctest::Approx(0.012));
    CHECK(c.relaxationLengthLongitudinal == doctest::Approx(0.35));
    CHECK(c.relaxationLengthLateral == doctest::Approx(0.45));
    CHECK(c.pDx2 == doctest::Approx(MagicFormulaCoefficients{}.pDx2)); // untouched default

    // A parse followed by a model evaluation works end to end.
    MagicFormulaTire tire(c);
    TireState state;
    state.normalLoad = c.nominalLoad;
    state.slipRatio = 0.1;
    CHECK(tire.computeForces(state).longitudinal > 0.0);
}
