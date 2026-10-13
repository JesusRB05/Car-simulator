// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <istream>
#include <string>

#include "physics/PhysicsTypes.hpp"
#include "vehicle/ITireModel.hpp"

namespace sim::vehicle {

using physics::Real;

// Pacejka Magic Formula 6.x pure-slip tyre. The coefficient set mirrors the
// subset of a `.tir` file needed for Fx(kappa) and Fy(alpha); the loader for the
// full file lands with P2.2. All coefficients refer to the nominal load Fz0.
//
// Sign conventions (matching TireState): a positive slip ratio (driving)
// produces a positive longitudinal force (along the wheel forward axis); a
// positive slip angle produces a negative lateral force, i.e. the tyre pushes
// against the slip.
struct MagicFormulaCoefficients {
    Real nominalLoad = 4000.0; // Fz0 (N)

    // Longitudinal (pure slip).
    Real pCx1 = 1.6;   // shape
    Real pDx1 = 1.4;   // peak friction mu_x at the nominal load
    Real pDx2 = -0.15; // load sensitivity of the peak
    Real pEx1 = 0.3;   // curvature at the peak
    Real pKx1 = 24.0;  // slip stiffness Kxkappa = Fz (pKx1 + pKx2 dfz) exp(pKx3 dfz)
    Real pKx2 = 8.0;
    Real pKx3 = 0.0;

    // Lateral (pure slip).
    Real pCy1 = 1.3; // shape
    Real pDy1 = 1.4; // peak friction mu_y at the nominal load
    Real pDy2 = -0.15;
    Real pEy1 = -0.3;
    Real pKy1 = 15.0; // cornering stiffness Kya = pKy1 Fz0 sin(2 atan(Fz / (pKy2 Fz0)))
    Real pKy2 = 2.5;

    Real radius = 0.31; // unloaded tyre radius (m)
};

class MagicFormulaTire : public ITireModel {
public:
    MagicFormulaTire() = default;
    explicit MagicFormulaTire(const MagicFormulaCoefficients& coefficients) : coefficients_(coefficients) {}

    const MagicFormulaCoefficients& coefficients() const { return coefficients_; }

    TireForces computeForces(const TireState& state) const override;
    Real unloadedRadius() const override { return coefficients_.radius; }

private:
    MagicFormulaCoefficients coefficients_{};
};

// Parses the pure-slip subset of a TNO `.tir` file: section headers in square
// brackets, `KEY = value` lines and `!`/`$` comments. Unknown keys and sections
// are ignored, so a full `.tir` file parses without complaint; keys that are
// absent keep their default. The value's first whitespace-delimited token is
// used, which matches the `.tir` layout (`PCX1 = 1.6  $ comment`).
MagicFormulaCoefficients parseMagicFormulaCoefficients(std::istream& input);

// Loads coefficients from a `.tir` file. Returns the defaults if it cannot be
// opened.
MagicFormulaCoefficients loadMagicFormulaCoefficients(const std::string& path);

} // namespace sim::vehicle
