// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "vehicle/IAerodynamics.hpp"

namespace sim::vehicle {

using physics::Real;
using physics::Vec3;

// Air density from the ideal gas law, rho = p / (R_sp * T), with the specific
// gas constant of dry air by default (J/(kg*K)).
Real airDensity(Real pressure, Real temperature, Real specificGasConstant = 287.05);

// Loads from the constant-coefficient model, exposed on its own so it can be
// tested without the interface.
AeroLoads computeAerodynamicLoads(const Vec3& airVelocityChassis, Real density, Real dragCoefficient,
                                  Real liftCoefficient, Real frontalArea, const Vec3& centreOfPressure,
                                  const Vec3& referencePoint);

// Aerodynamic model of the body (PHYSICS_PLAN Section 6.3). Drag and lift follow
// 1/2 rho C A v^2 with the air velocity v measured relative to the chassis; a
// positive lift coefficient is downforce. The loads act at the centre of pressure
// and the moment is taken about the reference point.
//
// Coefficients here are constant; a map of ride height, yaw and flap angle can be
// substituted by overriding computeLoads.
class AerodynamicModel : public IAerodynamics {
public:
    Real dragCoefficient = 0.30;
    Real liftCoefficient = 0.0; // + = downforce
    Real frontalArea = 2.0;     // A (m^2)
    Vec3 centreOfPressure{0.0, 0.0, 0.0};
    Vec3 referencePoint{0.0, 0.0, 0.0};

    AeroLoads computeLoads(const Vec3& airVelocityChassis, Real airDensity) const override;
};

} // namespace sim::vehicle
