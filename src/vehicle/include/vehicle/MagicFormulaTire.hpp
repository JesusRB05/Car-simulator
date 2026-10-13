// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <istream>
#include <string>

#include "physics/PhysicsTypes.hpp"
#include "vehicle/ITireModel.hpp"

namespace sim::vehicle {

using physics::Real;

// Scaling factors (the lambda of the `.tir` format). Applied to a base
// coefficient set they produce a variant (a different compound, a worn or a
// moulded tyre) without refitting. Each names the `.tir` key it maps to and all
// default to 1, which leaves the base set untouched.
struct TireScaling {
    Real fz0 = 1.0;     // LFZO, nominal (rated) load
    Real muX = 1.0;     // LMUX, longitudinal peak friction
    Real muY = 1.0;     // LMUY, lateral peak friction
    Real kxKappa = 1.0; // LKXK, longitudinal slip stiffness
    Real kyAlpha = 1.0; // LKYC, cornering stiffness
    Real cx = 1.0;      // LCX, longitudinal shape
    Real cy = 1.0;      // LCY, lateral shape
    Real ex = 1.0;      // LEX, longitudinal curvature
    Real ey = 1.0;      // LEY, lateral curvature
    Real hx = 1.0;      // LHX, longitudinal horizontal shift
    Real hy = 1.0;      // LHY, lateral horizontal shift
    Real vx = 1.0;      // LVX, longitudinal vertical shift
    Real vy = 1.0;      // LVY, lateral vertical shift
};

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

    // Camber (inclination gamma). It reduces the peaks and the cornering
    // stiffness, and adds a camber thrust as a lateral force shift.
    Real pDx3 = 0.0; // gamma^2 reduction of the longitudinal peak
    Real pDy3 = 0.0; // gamma^2 reduction of the lateral peak
    Real pKy3 = 0.0; // |gamma| reduction of the cornering stiffness

    // Pure-slip horizontal and vertical shifts S_H, S_V (MF 6.x subset). The
    // horizontal shifts move the slip at which the force vanishes, and the
    // vertical shifts add a force that survives at zero slip (ply steer and
    // camber thrust). Zero by default, which is correct for a symmetric tyre.
    Real pHx1 = 0.0;
    Real pHx2 = 0.0;
    Real pVx1 = 0.0;
    Real pVx2 = 0.0;
    Real pHy1 = 0.0;
    Real pHy2 = 0.0;
    Real pVy1 = 0.0;
    Real pVy2 = 0.0;
    Real pVy3 = 0.0; // camber thrust (times tan gamma)

    // Pneumatic trail, the source of the self-aligning moment Mz = -t Fy.
    Real qBz1 = 8.0;   // trail stiffness factor
    Real qCz1 = 1.1;   // trail shape
    Real qDz1 = 0.12;  // trail scale at the nominal load (m)
    Real qDz2 = -0.05; // trail load sensitivity
    Real qEz1 = -1.0;  // trail curvature

    // Residual self-aligning torque Mzr = Dr cos(Cr atan(Br alpha_r)) cos(alpha),
    // with alpha_r = alpha + SHr, Br = qBr1 + qBr2 dfz, Dr = Fz R0 (qDr1 + qDr2 dfz).
    // It adds a small, even-in-alpha torque that survives as the trail vanishes.
    // Zero by default, which is correct for a symmetric tyre.
    Real qCr1 = 1.0; // residual torque shape
    Real qBr1 = 0.1; // residual stiffness at the nominal load
    Real qBr2 = 0.0; // residual stiffness load sensitivity
    Real qDr1 = 0.0; // residual peak (as a fraction of Fz R0)
    Real qDr2 = 0.0; // residual peak load sensitivity
    Real qHr1 = 0.0; // residual horizontal shift (rad)

    // Overturning moment Mx = R0 Fz (qSx1 - qSx2 gamma + qSx3 Fy / Fz0).
    Real qSx1 = 0.0;
    Real qSx2 = 0.0;
    Real qSx3 = 0.0;

    // Rolling resistance as a speed-dependent coefficient Crr = C0 + C1 v + C2 v^2,
    // applied as the moment My = -Crr Fz Rl (Section 4.7).
    Real rollingResistanceC0 = 0.01;
    Real rollingResistanceC1 = 0.0;
    Real rollingResistanceC2 = 0.0;

    // Combined-slip weighting functions (MF 6.x subset). Ax is the shaping of the
    // longitudinal force with slip angle and Ay of the lateral force with slip
    // ratio: Gxa = cos(Cxa atan(Bxa alpha_s))/cos(Cxa atan(Bxa SHxa)), etc.
    Real rBx1 = 12.0;  // longitudinal weighting slope factor
    Real rBx2 = -10.0; // slip-ratio influence on the longitudinal slope
    Real rCx1 = 1.0;   // longitudinal weighting shape
    Real rHx1 = 0.0;   // longitudinal weighting shift (rad)
    Real rBy1 = 8.0;   // lateral weighting slope factor
    Real rBy2 = 8.0;   // slip-angle influence on the lateral slope
    Real rBy3 = 0.0;   // lateral weighting shift scale
    Real rCy1 = 1.0;   // lateral weighting shape
    Real rHy1 = 0.0;   // lateral weighting shift (dimensionless)

    // Slip-vector (friction-ellipse) form: the reference slip ratio and slip angle
    // at which the pure-slip forces peak, used to normalise the combined slip.
    Real peakSlipRatio = 0.12;
    Real peakSlipAngle = 0.13;

    // Transient relaxation lengths (Section 4.4), sigma in metres. They scale with
    // the tyre radius; the caller advances the relaxed slips by the fixed step.
    Real relaxationLengthLongitudinal = 0.3;
    Real relaxationLengthLateral = 0.4;

    // Scaling factors (the lambda of the `.tir` format), used to derive a variant
    // (another compound, a worn or a moulded tyre) from a base file without
    // refitting. They scale the whole coefficient set, so all default to 1.
    TireScaling scaling{};

    Real radius = 0.31; // unloaded tyre radius (m)
};

// Which combined-slip formulation the tyre uses. Both satisfy the friction-circle
// bound; the weighting functions follow the MF 6.x family, while the slip-vector
// form keeps the force aligned with the slip direction and stays well behaved far
// outside the fitted data (spins, drifts, loose surfaces).
enum class CombinedSlipModel {
    WeightingFunctions,
    SlipVector,
};

class MagicFormulaTire : public ITireModel {
public:
    MagicFormulaTire() = default;
    explicit MagicFormulaTire(const MagicFormulaCoefficients& coefficients) : coefficients_(coefficients) {}

    const MagicFormulaCoefficients& coefficients() const { return coefficients_; }

    // Selects the combined-slip formulation (default: weighting functions).
    void setCombinedSlipModel(CombinedSlipModel model) { combinedSlipModel_ = model; }
    CombinedSlipModel combinedSlipModel() const { return combinedSlipModel_; }

    TireForces computeForces(const TireState& state) const override;
    Real unloadedRadius() const override { return coefficients_.radius; }

private:
    MagicFormulaCoefficients coefficients_{};
    CombinedSlipModel combinedSlipModel_ = CombinedSlipModel::WeightingFunctions;
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

// First-order transient relaxation of the slip quantities (Section 4.4). A real
// tyre responds to a slip change over a rolled distance, not instantly, so the
// steady-state Magic Formula is driven by filtered slips. The filter is
// integrated exactly, which is stable for any dt and speed; at standstill the
// state is frozen and the low-speed model (4.5) takes over. The state is owned
// by the caller because ITireModel is a pure function of its inputs.
struct TireRelaxation {
    Real slipRatio = 0.0; // relaxed kappa'
    Real slipAngle = 0.0; // relaxed alpha'

    // Advances the relaxed slips towards the raw ones by dt and returns a
    // TireState carrying the relaxed slips (every other field is copied).
    TireState update(const MagicFormulaCoefficients& coefficients, const TireState& raw, Real dt);
};

} // namespace sim::vehicle
