// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <cmath>
#include <sstream>
#include <vector>

// Every interface header is included so that it is compile-checked here.
#include "vehicle/AntiRollBar.hpp"
#include "vehicle/Bushing.hpp"
#include "vehicle/DoubleWishbone.hpp"
#include "vehicle/IAerodynamics.hpp"
#include "vehicle/IBrakeSystem.hpp"
#include "vehicle/IDrivetrain.hpp"
#include "vehicle/ISuspension.hpp"
#include "vehicle/ITireModel.hpp"
#include "vehicle/IVehicleElectronics.hpp"
#include "vehicle/IVehicleModel.hpp"
#include "vehicle/MagicFormulaTire.hpp"
#include "vehicle/PlaceholderVehicle.hpp"
#include "vehicle/SpringDamper.hpp"
#include "vehicle/Steering.hpp"
#include "vehicle/TireBristle.hpp"
#include "vehicle/TireVertical.hpp"
#include "vehicle/WheelSpin.hpp"

using namespace sim::vehicle;

namespace {

constexpr Real kDt = 1.0 / 1000.0;

void run(IVehicleModel& model, const VehicleInput& input, int steps) {
    model.setInput(input);
    for (int i = 0; i < steps; ++i) {
        REQUIRE(model.step(kDt));
    }
}

// A purely linear longitudinal tyre, so the wheel run-up has an exact analytic
// solution to compare the implicit step against.
class LinearTire : public ITireModel {
public:
    explicit LinearTire(Real stiffness) : stiffness_(stiffness) {}
    TireForces computeForces(const TireState& state) const override {
        TireForces forces;
        forces.longitudinal = stiffness_ * state.slipRatio;
        return forces;
    }
    Real unloadedRadius() const override { return 0.31; }

private:
    Real stiffness_ = 0.0;
};

// A plausible right-front double wishbone, in the vehicle frame (x right, y up,
// -z forward). The numbers are round rather than measured; they only need to be
// a consistent, non-degenerate layout.
DoubleWishboneCorner sampleCorner() {
    DoubleWishboneCorner c;
    c.lowerFrontPivot = {0.35, 0.15, -0.25};
    c.lowerRearPivot = {0.35, 0.18, 0.25};
    c.upperFrontPivot = {0.45, 0.45, -0.20};
    c.upperRearPivot = {0.45, 0.47, 0.20};
    c.tieRodInner = {0.30, 0.25, 0.20};
    c.lowerBallJoint = {0.75, 0.20, 0.0};
    c.upperBallJoint = {0.72, 0.48, 0.0};
    c.tieRodOuter = {0.68, 0.26, 0.22};
    c.wheelCentre = {0.80, 0.31, 0.0};
    c.wheelAxle = {1.0, 0.0, 0.0};
    c.hasSpring = true;
    c.springUpper = {0.40, 0.70, 0.0};
    c.springLower = {0.62, 0.30, 0.0};
    return c;
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

TEST_CASE("camber modifies the tyre through thrust, peaks and shifts") {
    const MagicFormulaCoefficients c;
    const Real fz0 = c.nominalLoad;

    const auto state = [&](Real kappa, Real alpha, Real gamma) {
        TireState s;
        s.normalLoad = fz0;
        s.slipRatio = kappa;
        s.slipAngle = alpha;
        s.camber = gamma;
        return s;
    };

    SUBCASE("camber thrust adds a lateral force even at zero slip") {
        MagicFormulaCoefficients cc = c;
        cc.pVy3 = -0.3;
        MagicFormulaTire tire(cc);
        const Real gamma = 0.08;
        const TireForces f = tire.computeForces(state(0.0, 0.0, gamma));
        CHECK(f.lateral == doctest::Approx(fz0 * cc.pVy3 * std::tan(gamma)));
        CHECK(tire.computeForces(state(0.0, 0.0, 0.0)).lateral == doctest::Approx(0.0));
    }

    SUBCASE("camber reduces both peaks") {
        MagicFormulaCoefficients cc = c;
        cc.pDx3 = 0.5;
        cc.pDy3 = 0.4;
        MagicFormulaTire tire(cc);
        const Real gamma = 0.2;

        Real fxPeak = 0.0;
        Real fyPeak = 0.0;
        for (int i = 0; i <= 2000; ++i) {
            const Real x = 0.5 * static_cast<Real>(i) / 2000.0;
            fxPeak = std::max(fxPeak, tire.computeForces(state(x, 0.0, gamma)).longitudinal);
            fyPeak = std::max(fyPeak, std::abs(tire.computeForces(state(0.0, x, gamma)).lateral));
        }
        CHECK(fxPeak == doctest::Approx(c.pDx1 * fz0 * (1.0 - cc.pDx3 * gamma * gamma)).epsilon(0.01));
        CHECK(fyPeak == doctest::Approx(c.pDy1 * fz0 * (1.0 - cc.pDy3 * gamma * gamma)).epsilon(0.01));
    }

    SUBCASE("camber reduces the cornering stiffness") {
        MagicFormulaCoefficients cc = c;
        cc.pKy3 = 0.5;
        MagicFormulaTire tire(cc);
        const Real gamma = 0.2;
        const Real eps = 1e-5;
        const Real ky = c.pKy1 * fz0 * std::sin(2.0 * std::atan(fz0 / (c.pKy2 * fz0)));
        const TireForces f = tire.computeForces(state(0.0, eps, gamma));
        CHECK(-f.lateral / eps == doctest::Approx(ky * (1.0 - cc.pKy3 * gamma)).epsilon(1e-3));
    }

    SUBCASE("the horizontal shift moves the zero crossing") {
        MagicFormulaCoefficients cc = c;
        cc.pHx1 = 0.05;
        MagicFormulaTire tire(cc);
        CHECK(std::abs(tire.computeForces(state(-cc.pHx1, 0.0, 0.0)).longitudinal) < 1e-9);
    }
}

TEST_CASE("the .tir scaling factors scale the whole coefficient set") {
    const MagicFormulaCoefficients c;
    const Real fz0 = c.nominalLoad;

    const auto state = [&](Real kappa, Real alpha) {
        TireState s;
        s.normalLoad = fz0;
        s.slipRatio = kappa;
        s.slipAngle = alpha;
        return s;
    };

    SUBCASE("LMUX and LMUY scale the peaks") {
        MagicFormulaCoefficients cc = c;
        cc.scaling.muX = 1.2;
        cc.scaling.muY = 0.8;
        MagicFormulaTire tire(cc);
        Real fxPeak = 0.0;
        Real fyPeak = 0.0;
        for (int i = 0; i <= 2000; ++i) {
            const Real x = 0.5 * static_cast<Real>(i) / 2000.0;
            fxPeak = std::max(fxPeak, tire.computeForces(state(x, 0.0)).longitudinal);
            fyPeak = std::max(fyPeak, std::abs(tire.computeForces(state(0.0, x)).lateral));
        }
        CHECK(fxPeak == doctest::Approx(c.pDx1 * fz0 * cc.scaling.muX).epsilon(0.01));
        CHECK(fyPeak == doctest::Approx(c.pDy1 * fz0 * cc.scaling.muY).epsilon(0.01));
    }

    SUBCASE("LKXK and LKYC scale the slip stiffness") {
        MagicFormulaCoefficients cc = c;
        cc.scaling.kxKappa = 1.5;
        cc.scaling.kyAlpha = 0.6;
        MagicFormulaTire tire(cc);
        const Real eps = 1e-5;
        const Real kx = fz0 * c.pKx1; // dfz = 0
        const Real ky = c.pKy1 * fz0 * std::sin(2.0 * std::atan(1.0 / c.pKy2));
        CHECK(tire.computeForces(state(eps, 0.0)).longitudinal / eps == doctest::Approx(kx * 1.5).epsilon(1e-3));
        CHECK(-tire.computeForces(state(0.0, eps)).lateral / eps == doctest::Approx(ky * 0.6).epsilon(1e-3));
    }

    SUBCASE("LFZO rescales the nominal load used by dfz") {
        MagicFormulaCoefficients cc = c;
        cc.scaling.fz0 = 2.0; // rated load doubles, so dfz = -0.5 at the old load
        MagicFormulaTire tire(cc);
        const Real eps = 1e-5;
        const Real slope = tire.computeForces(state(eps, 0.0)).longitudinal / eps;
        CHECK(slope == doctest::Approx(fz0 * (c.pKx1 + c.pKx2 * (-0.5))).epsilon(1e-4));
    }
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

TEST_CASE("the bristle model holds at standstill and saturates on slip") {
    const TireBristleCoefficients c;
    const Real fz = 4000.0;
    const Real fs = c.staticFriction * fz;
    const Real fc = c.kineticFriction * fz;
    const Real dt = 1.0 / 1000.0;

    SUBCASE("a lifted tyre has no bristle state") {
        TireBristleState st;
        st.longitudinal = 0.01;
        const TireBristleResult r = updateTireBristle(c, 0.0, 0.05, 0.0, dt, st);
        CHECK(r.longitudinal == doctest::Approx(0.0));
        CHECK(r.lateral == doctest::Approx(0.0));
        CHECK(st.longitudinal == doctest::Approx(0.0));
    }

    SUBCASE("the sliding force opposes the slip and sits between kinetic and static") {
        TireBristleState st;
        TireBristleResult r;
        for (int i = 0; i < 5000; ++i) {
            r = updateTireBristle(c, fz, 0.05, 0.0, dt, st);
        }
        CHECK(r.longitudinal < 0.0);
        CHECK(std::abs(r.longitudinal) > fc);
        CHECK(std::abs(r.longitudinal) <= fs);
    }

    SUBCASE("a deflected bristle keeps a holding force at zero slip (stiction)") {
        TireBristleState st;
        TireBristleResult r;
        for (int i = 0; i < 500; ++i) {
            r = updateTireBristle(c, fz, 0.02, 0.0, dt, st);
        }
        const Real held = r.longitudinal;
        CHECK(held < 0.0);

        // The patch is now still, but the stored deflection keeps resisting; this
        // is what holds a parked car in place.
        const TireBristleResult still = updateTireBristle(c, fz, 0.0, 0.0, dt, st);
        CHECK(still.longitudinal < 0.0);
        CHECK(std::abs(still.longitudinal) > 0.9 * std::abs(held));
    }

    SUBCASE("the blend weight falls from one at rest to zero at blendSpeed") {
        TireBristleState st;
        CHECK(updateTireBristle(c, fz, 0.0, 0.0, dt, st).blend == doctest::Approx(1.0));
        CHECK(updateTireBristle(c, fz, c.blendSpeed, 0.0, dt, st).blend == doctest::Approx(0.0));

        Real previous = 1.1;
        for (int i = 0; i <= 10; ++i) {
            const Real speed = c.blendSpeed * static_cast<Real>(i) / 10.0;
            TireBristleState sample;
            const Real w = updateTireBristle(c, fz, speed, 0.0, dt, sample).blend;
            CHECK(w <= previous + 1e-12);
            previous = w;
        }
    }
}

TEST_CASE("the wheel run-up follows the analytic exponential") {
    const Real dt = 1.0 / 1000.0;
    const Real re = 0.31;
    const Real vcx = 30.0;
    const Real stiffness = 2000.0;
    const Real inertia = 10.0;
    const Real driveTorque = 300.0;
    LinearTire tire(stiffness);

    WheelSpinCoefficients c;
    c.inertia = inertia;

    WheelSpinInput input;
    input.contactSpeed = vcx;
    input.effectiveRadius = re;
    input.driveTorque = driveTorque;

    TireState context;
    context.normalLoad = 4000.0;
    context.contactSpeed = vcx;

    WheelSpinState st;
    st.spin = vcx / re; // starts at zero slip

    const Real rate = re * re * stiffness / (vcx * inertia);
    const Real omegaStar = vcx / re + driveTorque * vcx / (re * re * stiffness);

    const int steps = 1000;
    for (int i = 0; i < steps; ++i) {
        stepWheelSpin(c, tire, context, input, dt, st);
    }
    const Real expected = omegaStar + (vcx / re - omegaStar) * std::exp(-rate * steps * dt);
    CHECK(st.spin == doctest::Approx(expected).epsilon(0.01));
    CHECK(st.spin > vcx / re); // the wheel sped up into positive slip
}

TEST_CASE("a braked wheel locks without oscillating") {
    const Real dt = 1.0 / 1000.0;
    MagicFormulaTire tire;

    WheelSpinCoefficients c;
    c.inertia = 1.2;

    WheelSpinInput input;
    input.contactSpeed = 20.0;
    input.effectiveRadius = tire.unloadedRadius();
    input.brakeTorque = 3000.0;

    TireState context;
    context.normalLoad = 4000.0;
    context.contactSpeed = 20.0;

    WheelSpinState st;
    st.spin = 80.0;

    Real previous = st.spin;
    int reversals = 0;
    for (int i = 0; i < 2000; ++i) {
        stepWheelSpin(c, tire, context, input, dt, st);
        CHECK(st.spin >= 0.0);
        if (st.spin > previous + 1e-9) {
            ++reversals;
        }
        previous = st.spin;
    }
    CHECK(reversals == 0);
    CHECK(st.locked);
    CHECK(st.spin == doctest::Approx(0.0));
}

TEST_CASE("the double wishbone closure solves and reports camber and toe") {
    const DoubleWishboneCorner corner = sampleCorner();
    const DoubleWishbone suspension(corner);

    const std::array<Vec3, 5> q = {corner.lowerBallJoint, corner.lowerBallJoint, corner.upperBallJoint,
                                   corner.upperBallJoint, corner.tieRodOuter};
    const std::array<Vec3, 5> p = {corner.lowerFrontPivot, corner.lowerRearPivot, corner.upperFrontPivot,
                                   corner.upperRearPivot, corner.tieRodInner};

    SUBCASE("the design pose is the identity") {
        const CornerState s = suspension.solveForTravel(0.0);
        CHECK(s.converged);
        CHECK(glm::length(s.translation) < 1e-9);
        CHECK(s.camber == doctest::Approx(0.0).epsilon(1e-9));
        CHECK(s.toe == doctest::Approx(0.0).epsilon(1e-9));
    }

    SUBCASE("the closure holds the five link lengths over the travel range") {
        CornerState s = suspension.solveForTravel(0.0);
        const auto check = [&](Real travel) {
            INFO("travel=", travel);
            s = suspension.solveForTravel(travel, s);
            REQUIRE(s.converged);
            CHECK(s.wheelCentre.z == doctest::Approx(corner.wheelCentre.z + travel).epsilon(1e-9));
            for (int i = 0; i < 5; ++i) {
                const Vec3 moved = s.rotation * q[static_cast<std::size_t>(i)] + s.translation;
                CHECK(glm::length(moved - p[static_cast<std::size_t>(i)]) ==
                      doctest::Approx(suspension.linkLengths()[static_cast<std::size_t>(i)]).epsilon(1e-9));
            }
        };
        for (int k = 1; k <= 12; ++k) {
            check(0.005 * static_cast<Real>(k));
        }
        for (int k = 1; k <= 12; ++k) {
            check(-0.005 * static_cast<Real>(k));
        }
    }

    SUBCASE("the motion ratio matches a finite difference of the solved poses") {
        const Real travel = 0.03;
        const Real h = 1e-4;
        const CornerState low = suspension.solveForTravel(travel - h);
        const CornerState high = suspension.solveForTravel(travel + h, low);
        const Real fd = (high.springLength - low.springLength) / (2.0 * h);
        CHECK(suspension.motionRatio(travel) == doctest::Approx(fd).epsilon(1e-3));
        CHECK(suspension.motionRatio(travel) > 0.0);
    }

    SUBCASE("camber and toe read the wheel axis") {
        const Real gamma = 0.05;
        const CornerState c = suspension.pose(Vec3(0.0), Vec3(0.0, 0.0, gamma));
        CHECK(c.camber == doctest::Approx(gamma).epsilon(1e-9));
        CHECK(c.toe == doctest::Approx(0.0).epsilon(1e-9));

        const Real tau = 0.04;
        const CornerState t = suspension.pose(Vec3(0.0), Vec3(0.0, tau, 0.0));
        CHECK(t.toe == doctest::Approx(tau).epsilon(1e-9));
        CHECK(t.camber == doctest::Approx(0.0).epsilon(1e-9));
    }
}

TEST_CASE("the steering-axis geometry follows the ball joints") {
    DoubleWishboneCorner c = sampleCorner();
    c.lowerBallJoint = {0.70, 0.20, 0.10};
    c.upperBallJoint = {0.64, 0.50, 0.02};
    c.wheelCentre = {0.80, 0.31, 0.0};
    const DoubleWishbone suspension(c);

    const CornerState s = suspension.pose(Vec3(0.0), Vec3(0.0));
    CHECK(s.kingpinInclination == doctest::Approx(0.1973956).epsilon(1e-6));
    CHECK(s.caster == doctest::Approx(-0.2606023).epsilon(1e-6));
    CHECK(s.scrubRadius == doctest::Approx(-0.06).epsilon(1e-6));
    CHECK(s.mechanicalTrail == doctest::Approx(-0.1533333).epsilon(1e-6));
    CHECK(glm::length(s.steeringAxis - glm::normalize(c.upperBallJoint - c.lowerBallJoint)) < 1e-12);
}

TEST_CASE("the instant centres and roll centre come out of the arm lines") {
    SUBCASE("the front-view instant centre follows the arm lines") {
        const DoubleWishbone suspension(sampleCorner());
        const CornerState s = suspension.pose(Vec3(0.0), Vec3(0.0));
        CHECK(s.frontInstantCentre.x == doctest::Approx(21.77069).epsilon(1e-5));
        CHECK(s.frontInstantCentre.y == doctest::Approx(2.03931).epsilon(1e-5));
    }

    SUBCASE("the side-view instant centre sets the anti-dive") {
        DoubleWishboneCorner c = sampleCorner();
        c.lowerFrontPivot = {0.35, 0.20, -0.10};
        c.lowerRearPivot = {0.35, 0.20, 0.30};
        c.upperFrontPivot = {0.45, 0.46, -0.50};
        c.upperRearPivot = {0.45, 0.46, 0.10};
        const DoubleWishbone suspension(c);
        const CornerState s = suspension.pose(Vec3(0.0), Vec3(0.0));
        CHECK(s.sideInstantCentre.z == doctest::Approx(-2.8).epsilon(1e-6));
        CHECK(s.sideInstantCentre.y == doctest::Approx(0.20).epsilon(1e-6));
        CHECK(antiDiveFraction(s, 2.6, 0.5) == doctest::Approx(0.3714286).epsilon(1e-6));
    }

    SUBCASE("a symmetric pair meets on the centreline") {
        DoubleWishboneCorner left = sampleCorner();
        const auto mirror = [](Vec3& v) { v.x = -v.x; };
        mirror(left.lowerFrontPivot);
        mirror(left.lowerRearPivot);
        mirror(left.upperFrontPivot);
        mirror(left.upperRearPivot);
        mirror(left.tieRodInner);
        mirror(left.lowerBallJoint);
        mirror(left.upperBallJoint);
        mirror(left.tieRodOuter);
        mirror(left.wheelCentre);
        mirror(left.wheelAxle);
        mirror(left.springUpper);
        mirror(left.springLower);

        const DoubleWishbone rightSuspension(sampleCorner());
        const DoubleWishbone leftSuspension(left);
        const CornerState r = rightSuspension.pose(Vec3(0.0), Vec3(0.0));
        const CornerState l = leftSuspension.pose(Vec3(0.0), Vec3(0.0));
        const Vec3 rc = rollCentre(l, r);
        CHECK(std::abs(rc.x) < 1e-9);
        CHECK(rc.y == doctest::Approx(-0.077796).epsilon(1e-4));
    }
}

TEST_CASE("the spring, damper and stops give their own forces") {
    SUBCASE("the coil spring is linear with a preload") {
        SpringDamperCoefficients c;
        c.stiffness = 50000.0;
        c.restLength = 0.3;
        c.preload = 200.0;

        CHECK(computeSpringDamperForces(c, {0.28, 0.0}).spring == doctest::Approx(1200.0));
        CHECK(computeSpringDamperForces(c, {0.30, 0.0}).spring == doctest::Approx(200.0));
        CHECK(computeSpringDamperForces(c, {0.32, 0.0}).spring == doctest::Approx(-800.0));
    }

    SUBCASE("the damper is asymmetric between bump and rebound") {
        SpringDamperCoefficients c;
        c.dampingCompression = 3000.0;
        c.dampingRebound = 6000.0;

        CHECK(computeSpringDamperForces(c, {0.3, 0.1}).damper == doctest::Approx(300.0));
        CHECK(computeSpringDamperForces(c, {0.3, -0.1}).damper == doctest::Approx(-600.0));
    }

    SUBCASE("the damper table interpolates and clamps") {
        SpringDamperCoefficients c;
        c.damperTable.speeds = {-0.5, 0.0, 0.5};
        c.damperTable.forces = {-600.0, 0.0, 300.0};

        CHECK(computeSpringDamperForces(c, {0.3, 0.25}).damper == doctest::Approx(150.0));
        CHECK(computeSpringDamperForces(c, {0.3, 1.0}).damper == doctest::Approx(300.0));
        CHECK(computeSpringDamperForces(c, {0.3, -1.0}).damper == doctest::Approx(-600.0));
    }

    SUBCASE("the bump stop is progressive and one-sided") {
        SpringDamperCoefficients c;
        c.hasBumpStop = true;
        c.bumpStopLength = 0.25;
        c.bumpStopStiffness = 1.0e6;
        c.bumpStopExponent = 2.0;

        CHECK(computeSpringDamperForces(c, {0.26, 0.0}).bumpStop == doctest::Approx(0.0));
        CHECK(computeSpringDamperForces(c, {0.24, 0.0}).bumpStop == doctest::Approx(100.0));
        CHECK(computeSpringDamperForces(c, {0.23, 0.0}).bumpStop == doctest::Approx(400.0));

        c.bumpStopDamping = 5000.0;
        CHECK(computeSpringDamperForces(c, {0.24, 0.1}).bumpStop == doctest::Approx(600.0));
    }

    SUBCASE("the rebound stop pulls against extension") {
        SpringDamperCoefficients c;
        c.hasReboundStop = true;
        c.reboundStopLength = 0.35;
        c.reboundStopStiffness = 2.0e6;
        c.reboundStopExponent = 2.0;

        CHECK(computeSpringDamperForces(c, {0.34, 0.0}).reboundStop == doctest::Approx(0.0));
        CHECK(computeSpringDamperForces(c, {0.37, 0.0}).reboundStop == doctest::Approx(-800.0));

        c.reboundStopDamping = 4000.0;
        CHECK(computeSpringDamperForces(c, {0.37, 0.1}).reboundStop == doctest::Approx(-1200.0));
    }

    SUBCASE("the total sums the four contributions") {
        SpringDamperCoefficients c;
        c.stiffness = 50000.0;
        c.restLength = 0.3;
        c.dampingCompression = 3000.0;
        c.hasBumpStop = true;
        c.bumpStopLength = 0.25;
        c.bumpStopStiffness = 1.0e6;
        c.bumpStopExponent = 2.0;

        const SpringDamperForces f = computeSpringDamperForces(c, {0.24, 0.1});
        CHECK(f.spring == doctest::Approx(50000.0 * (0.3 - 0.24)));
        CHECK(f.damper == doctest::Approx(300.0));
        CHECK(f.bumpStop == doctest::Approx(100.0));
        CHECK(f.total() == doctest::Approx(f.spring + f.damper + f.bumpStop));
    }
}

TEST_CASE("the anti-roll bar answers only a travel difference") {
    const Real kTheta = 20000.0;
    const Real tA = 1.0;

    const AntiRollBarForces level = computeAntiRollBarForces(kTheta, tA, 0.02, 0.02);
    CHECK(level.left == doctest::Approx(0.0));
    CHECK(level.right == doctest::Approx(0.0));

    const AntiRollBarForces rolled = computeAntiRollBarForces(kTheta, tA, 0.03, 0.01);
    CHECK(rolled.left == doctest::Approx(-400.0));
    CHECK(rolled.right == doctest::Approx(400.0));
    CHECK(rolled.left + rolled.right == doctest::Approx(0.0)); // a couple, no net force

    CHECK(computeAntiRollBarForces(kTheta, 0.0, 0.03, 0.01).left == doctest::Approx(0.0));
}

TEST_CASE("the bushing restores its six degrees of freedom per axis") {
    BushingCoefficients c;
    c.translationalStiffness = {1.0e5, 2.0e5, 3.0e5};
    c.rotationalStiffness = {100.0, 200.0, 300.0};

    const BushingForces f = computeBushingForces(c, {{0.01, 0.02, 0.03}, {0.1, 0.2, 0.3}});
    CHECK(f.force.x == doctest::Approx(-1000.0));
    CHECK(f.force.y == doctest::Approx(-4000.0));
    CHECK(f.force.z == doctest::Approx(-9000.0));
    CHECK(f.moment.x == doctest::Approx(-10.0));
    CHECK(f.moment.y == doctest::Approx(-40.0));
    CHECK(f.moment.z == doctest::Approx(-90.0));

    const BushingForces rest = computeBushingForces(c, {});
    CHECK(glm::length(rest.force) == doctest::Approx(0.0));
    CHECK(glm::length(rest.moment) == doctest::Approx(0.0));
}

TEST_CASE("a quarter car rides and damps at its analytic rate") {
    const Real m = 300.0;
    SpringDamperCoefficients c;
    c.stiffness = 30000.0;
    c.restLength = 0.30;
    c.dampingCompression = 2000.0;
    c.dampingRebound = 2000.0;

    const Real omegaN = std::sqrt(c.stiffness / m);
    const Real zeta = c.dampingCompression / (2.0 * std::sqrt(c.stiffness * m));
    const Real omegaD = omegaN * std::sqrt(1.0 - zeta * zeta);

    const Real dt = 1.0e-5;
    Real x = 0.05; // compression (m), positive in bump
    Real v = 0.0;
    std::vector<Real> zeros;
    std::vector<Real> peaks;
    const int steps = 200000; // 2 s
    for (int i = 0; i < steps; ++i) {
        const SpringDamperForces f = computeSpringDamperForces(c, {c.restLength - x, v});
        const Real a = -f.total() / m;
        const Real vNext = v + a * dt;
        const Real xNext = x + vNext * dt;
        if (v > 0.0 && vNext <= 0.0) {
            peaks.push_back(x); // a positive peak of the response
        }
        if (x > 0.0 && xNext <= 0.0) {
            zeros.push_back(static_cast<Real>(i) * dt); // a downward zero crossing
        }
        x = xNext;
        v = vNext;
    }

    REQUIRE(zeros.size() >= 3);
    REQUIRE(peaks.size() >= 2);

    const Real measuredOmega = 2.0 * 3.14159265358979323846 / (zeros[1] - zeros[0]);
    CHECK(measuredOmega == doctest::Approx(omegaD).epsilon(2.0e-3));

    const Real logDecrement = std::log(peaks[0] / peaks[1]);
    const Real analytic = 2.0 * 3.14159265358979323846 * zeta / std::sqrt(1.0 - zeta * zeta);
    CHECK(logDecrement == doctest::Approx(analytic).epsilon(2.0e-2));
}

TEST_CASE("the steering gives Ackermann angles and a column torque") {
    const Real pi = 3.14159265358979323846;
    const AckermannGeometry geom{1.5, 2.6};

    SUBCASE("the outer wheel satisfies the Ackermann condition") {
        const Real inner = 20.0 * pi / 180.0;
        const Real outer = ackermannOuterAngle(geom, inner);
        CHECK(1.0 / std::tan(outer) - 1.0 / std::tan(inner) == doctest::Approx(geom.track / geom.wheelbase));
        CHECK(outer < inner); // the outer wheel steers less
    }

    SUBCASE("the Ackermann percentage blends parallel to full") {
        const Real inner = 0.3;
        const Real ideal = ackermannOuterAngle(geom, inner);
        CHECK(steerAngles(geom, inner, 0.0).outer == doctest::Approx(inner));
        CHECK(steerAngles(geom, inner, 1.0).outer == doctest::Approx(ideal));
        CHECK(steerAngles(geom, inner, 0.5).outer == doctest::Approx(0.5 * (inner + ideal)));
    }

    SUBCASE("the rack ratio scales the steering wheel angle") {
        CHECK(roadWheelAngle(15.0, 0.5 * pi) == doctest::Approx(0.5 * pi / 15.0));
        CHECK(roadWheelAngle(0.0, 1.0) == doctest::Approx(0.0));
    }

    SUBCASE("the column torque sums the aligning torques and the assist") {
        const std::vector<WheelSteerLoad> wheels{{20.0, 3000.0, 0.03, 0.02}, {20.0, 3000.0, 0.03, 0.02}};
        // Each wheel: 20 + 3000*(0.03 + 0.02) = 170 N*m, two of them 340.
        CHECK(steeringColumnTorque(wheels, 0.9, -100.0, 5.0) == doctest::Approx(340.0 / 0.9 - 100.0 + 5.0));
        // A zero efficiency falls back to 1 instead of dividing by zero.
        CHECK(steeringColumnTorque(wheels, 0.0, 0.0, 0.0) == doctest::Approx(340.0));
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
                           "PDX3 = 0.4\n"
                           "PVY3 = -0.25\n"
                           "PHX1 = 0.02\n"
                           "LMUX = 1.15\n"
                           "LKYC = 0.9\n"
                           "LFZO = 1.0\n"
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
    CHECK(c.pDx3 == doctest::Approx(0.4));
    CHECK(c.pVy3 == doctest::Approx(-0.25));
    CHECK(c.pHx1 == doctest::Approx(0.02));
    CHECK(c.scaling.muX == doctest::Approx(1.15));
    CHECK(c.scaling.kyAlpha == doctest::Approx(0.9));
    CHECK(c.scaling.fz0 == doctest::Approx(1.0));
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
