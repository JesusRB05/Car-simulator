// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/MagicFormulaTire.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace sim::vehicle {
namespace {

// y(x) = D sin(C atan(Bx - E (Bx - atan(Bx)))). Odd in x, with slope BCD at the
// origin and peak |y| = D.
Real magic(Real b, Real c, Real d, Real e, Real x) {
    const Real bx = b * x;
    return d * std::sin(c * std::atan(bx - e * (bx - std::atan(bx))));
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

    // Longitudinal, from the slip ratio kappa.
    const Real cx = c.pCx1;
    const Real dx = (c.pDx1 + c.pDx2 * dfz) * fz;
    const Real kx = fz * (c.pKx1 + c.pKx2 * dfz) * std::exp(c.pKx3 * dfz);
    const Real bx = (cx * dx != 0.0) ? kx / (cx * dx) : 0.0;
    Real fx = magic(bx, cx, dx, c.pEx1, state.slipRatio);

    // Lateral, from the slip angle alpha. pKy1 > 0 gives a cornering stiffness
    // and the force acts against the slip, hence the sign.
    const Real cy = c.pCy1;
    const Real dy = (c.pDy1 + c.pDy2 * dfz) * fz;
    const Real ky = c.pKy1 * fz0 * std::sin(2.0 * std::atan(fz / (c.pKy2 * fz0)));
    const Real by = (cy * dy != 0.0) ? ky / (cy * dy) : 0.0;
    Real fy = -magic(by, cy, dy, c.pEy1, state.slipAngle);

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

    forces.longitudinal = fx;
    forces.lateral = fy;
    return forces;
}

} // namespace sim::vehicle
