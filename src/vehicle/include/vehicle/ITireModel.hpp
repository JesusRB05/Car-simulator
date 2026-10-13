// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Operating point of one tyre contact patch. Slip definitions follow ISO 8855
// (SAE-style signs mapped onto the project frame: +X right, +Y up, -Z forward).
struct TireState
{
    Real slipRatio = 0.0;           // longitudinal slip kappa, dimensionless (+ = driving)
    Real slipAngle = 0.0;           // lateral slip alpha (rad)
    Real normalLoad = 0.0;          // vertical load Fz (N), >= 0
    Real camber = 0.0;              // inclination angle gamma (rad)
    Real frictionCoefficient = 1.0; // road surface friction scale mu, dimensionless
    Real contactSpeed = 0.0;        // longitudinal contact-patch speed (m/s), for low-speed handling
};

// Forces and moments produced at the contact patch, in the wheel frame.
struct TireForces
{
    Real longitudinal = 0.0;            // Fx (N), + = along the wheel forward axis
    Real lateral = 0.0;                 // Fy (N), + = towards the wheel's left
    Real aligningMoment = 0.0;          // Mz (N*m) about the wheel vertical axis
    Real overturningMoment = 0.0;       // Mx (N*m)
    Real rollingResistanceMoment = 0.0; // My (N*m)
};

// Tyre model (e.g. Pacejka Magic Formula, brush model). Must be a pure,
// deterministic function of its inputs; transient relaxation state, if any, is
// owned by the caller and advanced with the fixed physics timestep.
class ITireModel
{
public:
    virtual ~ITireModel() = default;

    // Computes contact-patch forces for the given operating point.
    virtual TireForces computeForces(const TireState& state) const = 0;

    // Unloaded tyre radius (m).
    virtual Real unloadedRadius() const = 0;
};

} // namespace sim::vehicle
