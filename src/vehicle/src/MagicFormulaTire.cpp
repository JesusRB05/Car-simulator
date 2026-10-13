// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/MagicFormulaTire.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace sim::vehicle {
namespace {

constexpr Real kPi = 3.14159265358979323846;

// y(x) = D sin(C atan(Bx - E (Bx - atan(Bx)))). Odd in x, with slope BCD at the
// origin and peak |y| = D.
Real magic(Real b, Real c, Real d, Real e, Real x) {
    const Real bx = b * x;
    return d * std::sin(c * std::atan(bx - e * (bx - std::atan(bx))));
}

// The argument inside atan, Bx - E (Bx - atan Bx).
Real magicArgument(Real b, Real e, Real x) {
    const Real bx = b * x;
    return bx - e * (bx - std::atan(bx));
}

} // namespace

MagicFormulaCoefficients parseMagicFormulaCoefficients(std::istream& input) {
    MagicFormulaCoefficients c;
    std::string line;
    while (std::getline(input, line)) {
        const std::size_t comment = line.find_first_of("!$");
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        std::istringstream fields(line);
        std::string key;
        std::string equals;
        Real value = 0.0;
        if (!(fields >> key >> equals >> value) || equals != "=") {
            continue; // blank line, section header or something we do not read
        }
        if (key == "FNOMIN") {
            c.nominalLoad = value;
        } else if (key == "UNLOADED_RADIUS") {
            c.radius = value;
        } else if (key == "PCX1") {
            c.pCx1 = value;
        } else if (key == "PDX1") {
            c.pDx1 = value;
        } else if (key == "PDX2") {
            c.pDx2 = value;
        } else if (key == "PEX1") {
            c.pEx1 = value;
        } else if (key == "PKX1") {
            c.pKx1 = value;
        } else if (key == "PKX2") {
            c.pKx2 = value;
        } else if (key == "PKX3") {
            c.pKx3 = value;
        } else if (key == "PCY1") {
            c.pCy1 = value;
        } else if (key == "PDY1") {
            c.pDy1 = value;
        } else if (key == "PDY2") {
            c.pDy2 = value;
        } else if (key == "PEY1") {
            c.pEy1 = value;
        } else if (key == "PKY1") {
            c.pKy1 = value;
        } else if (key == "PKY2") {
            c.pKy2 = value;
        } else if (key == "QBZ1") {
            c.qBz1 = value;
        } else if (key == "QCZ1") {
            c.qCz1 = value;
        } else if (key == "QDZ1") {
            c.qDz1 = value;
        } else if (key == "QDZ2") {
            c.qDz2 = value;
        } else if (key == "QEZ1") {
            c.qEz1 = value;
        } else if (key == "QCR1") {
            c.qCr1 = value;
        } else if (key == "QBR1") {
            c.qBr1 = value;
        } else if (key == "QBR2") {
            c.qBr2 = value;
        } else if (key == "QDR1") {
            c.qDr1 = value;
        } else if (key == "QDR2") {
            c.qDr2 = value;
        } else if (key == "QHR1") {
            c.qHr1 = value;
        } else if (key == "QSX1") {
            c.qSx1 = value;
        } else if (key == "QSX2") {
            c.qSx2 = value;
        } else if (key == "QSX3") {
            c.qSx3 = value;
        } else if (key == "RBX1") {
            c.rBx1 = value;
        } else if (key == "RBX2") {
            c.rBx2 = value;
        } else if (key == "RCX1") {
            c.rCx1 = value;
        } else if (key == "RHX1") {
            c.rHx1 = value;
        } else if (key == "RBY1") {
            c.rBy1 = value;
        } else if (key == "RBY2") {
            c.rBy2 = value;
        } else if (key == "RBY3") {
            c.rBy3 = value;
        } else if (key == "RCY1") {
            c.rCy1 = value;
        } else if (key == "RHY1") {
            c.rHy1 = value;
        } else if (key == "ROLLING_RESISTANCE") {
            c.rollingResistanceC0 = value;
        } else if (key == "ROLLING_RESISTANCE_C1") {
            c.rollingResistanceC1 = value;
        } else if (key == "ROLLING_RESISTANCE_C2") {
            c.rollingResistanceC2 = value;
        } else if (key == "SIGMA_X") {
            c.relaxationLengthLongitudinal = value;
        } else if (key == "SIGMA_Y") {
            c.relaxationLengthLateral = value;
        }
    }
    return c;
}

MagicFormulaCoefficients loadMagicFormulaCoefficients(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        return MagicFormulaCoefficients{};
    }
    return parseMagicFormulaCoefficients(file);
}

TireForces MagicFormulaTire::computeForces(const TireState& state) const {
    const MagicFormulaCoefficients& c = coefficients_;
    TireForces forces;

    const Real fz = std::max(0.0, state.normalLoad);
    if (fz == 0.0) {
        return forces; // a lifted tyre develops no force
    }

    const Real fz0 = c.nominalLoad;
    const Real dfz = (fz - fz0) / fz0;

    // Pure-slip forces. fx0 uses the longitudinal stiffness kx; fy0 uses the
    // cornering stiffness ky and acts against the slip, hence the sign.
    const Real cx = c.pCx1;
    const Real dx = (c.pDx1 + c.pDx2 * dfz) * fz;
    const Real kx = fz * (c.pKx1 + c.pKx2 * dfz) * std::exp(c.pKx3 * dfz);
    const Real bx = (cx * dx != 0.0) ? kx / (cx * dx) : 0.0;
    const Real fx0 = magic(bx, cx, dx, c.pEx1, state.slipRatio);

    const Real cy = c.pCy1;
    const Real dy = (c.pDy1 + c.pDy2 * dfz) * fz;
    const Real ky = c.pKy1 * fz0 * std::sin(2.0 * std::atan(fz / (c.pKy2 * fz0)));
    const Real by = (cy * dy != 0.0) ? ky / (cy * dy) : 0.0;
    const Real fy0 = -magic(by, cy, dy, c.pEy1, state.slipAngle);

    // Combined slip (Section 4.3). Both forms reduce to the pure-slip force when
    // the other slip is zero; the friction bound below still applies to both.
    Real fx = fx0;
    Real fy = fy0;
    if (combinedSlipModel_ == CombinedSlipModel::WeightingFunctions) {
        // Gxa(alpha) scales the longitudinal force; the slope depends on kappa.
        const Real bxa = c.rBx1 * std::cos(std::atan(c.rBx2 * state.slipRatio));
        const Real alphaShifted = state.slipAngle + c.rHx1;
        const Real gxa =
            (bxa != 0.0) ? std::cos(c.rCx1 * std::atan(bxa * alphaShifted)) / std::cos(c.rCx1 * std::atan(bxa * c.rHx1))
                         : 1.0;

        // Gyk(kappa) scales the lateral force; the slope depends on alpha.
        const Real byk = c.rBy1 * std::cos(std::atan(c.rBy2 * (state.slipAngle - c.rBy3)));
        const Real kappaShifted = state.slipRatio + c.rHy1;
        const Real gyk =
            (byk != 0.0) ? std::cos(c.rCy1 * std::atan(byk * kappaShifted)) / std::cos(c.rCy1 * std::atan(byk * c.rHy1))
                         : 1.0;

        fx = gxa * fx0;
        fy = gyk * fy0;
    } else {
        // Slip-vector (friction-ellipse) form: a single resultant force aligned
        // with the normalised slip direction, well behaved far outside the data.
        const Real sx = (c.peakSlipRatio != 0.0) ? state.slipRatio / c.peakSlipRatio : 0.0;
        const Real tanAlpha = std::tan(state.slipAngle);
        const Real tanPeak = std::tan(c.peakSlipAngle);
        const Real sy = (tanPeak != 0.0) ? tanAlpha / tanPeak : 0.0;
        const Real s = std::sqrt(sx * sx + sy * sy);
        if (s > 0.0) {
            const Real muX = c.pDx1 + c.pDx2 * dfz;
            const Real muY = c.pDy1 + c.pDy2 * dfz;
            const Real dmu = std::max(muX, muY) * fz;
            // Resultant shape constant chosen so the peak sits at s = 1.
            const Real cs = 1.5;
            const Real bs = std::tan(0.5 * kPi / cs);
            const Real resultant = magic(bs, cs, dmu, 0.0, s);
            fx = resultant * (sx / s);
            fy = -resultant * (sy / s);
        } else {
            fx = 0.0;
            fy = 0.0;
        }
    }

    // Friction-circle bound so a combined (kappa, alpha) input never asks for
    // more grip than the peak available on this surface.
    const Real muX = c.pDx1 + c.pDx2 * dfz;
    const Real muY = c.pDy1 + c.pDy2 * dfz;
    const Real limit = std::max(0.0, std::max(muX, muY) * state.frictionCoefficient) * fz;
    const Real magnitude = std::sqrt(fx * fx + fy * fy);
    if (magnitude > limit && magnitude > 0.0) {
        const Real scale = limit / magnitude;
        fx *= scale;
        fy *= scale;
    }

    // Self-aligning moment from the pneumatic trail t(alpha); t0 and its shape
    // follow the Magic Formula so the trail falls to zero as the tyre saturates,
    // which is the classic self-aligning torque that feeds force feedback.
    const Real alphaT = state.slipAngle;
    const Real dtScale = c.qDz1 + c.qDz2 * dfz;
    const Real trail = dtScale * std::cos(c.qCz1 * std::atan(magicArgument(c.qBz1, c.qEz1, alphaT))) * std::cos(alphaT);
    const Real trailTorque = -trail * fy;

    // Residual torque: a small, even-in-alpha term that survives as the trail
    // vanishes. It is zero for a symmetric tyre (the default coefficients).
    const Real alphaR = state.slipAngle + c.qHr1;
    const Real br = c.qBr1 + c.qBr2 * dfz;
    const Real dr = fz * c.radius * (c.qDr1 + c.qDr2 * dfz);
    const Real residualTorque = dr * std::cos(c.qCr1 * std::atan(br * alphaR)) * std::cos(state.slipAngle);

    // Overturning moment Mx about the longitudinal axis (camber and lateral force).
    const Real mx = c.radius * fz * (c.qSx1 - c.qSx2 * state.camber + c.qSx3 * fy / fz0);

    // Rolling resistance Crr(v) Fz applied as a moment about the wheel axis. The
    // loaded radius is approximated by the unloaded radius (deflection is small).
    const Real crr = c.rollingResistanceC0 + c.rollingResistanceC1 * state.contactSpeed +
                     c.rollingResistanceC2 * state.contactSpeed * state.contactSpeed;
    const Real my = -crr * fz * c.radius;

    forces.longitudinal = fx;
    forces.lateral = fy;
    forces.aligningMoment = trailTorque + residualTorque;
    forces.overturningMoment = mx;
    forces.rollingResistanceMoment = my;
    return forces;
}

} // namespace sim::vehicle
