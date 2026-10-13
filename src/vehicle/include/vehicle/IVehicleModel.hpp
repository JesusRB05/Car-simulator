// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Quat;
using physics::Real;
using physics::Vec3;

// Driver controls fed into a vehicle model. All values are normalised.
struct VehicleInput
{
    Real throttle = 0.0; // [0, 1]
    Real brake = 0.0;    // [0, 1]
    Real steer = 0.0;    // [-1, 1], positive = steer left (counter-clockwise seen from +Y)
    bool handbrake = false;
};

// Top-level vehicle model contract consumed by the application, telemetry and
// the renderer.
//
// Frame convention (no exceptions anywhere in the project): right-handed world,
// +X right, +Y up, -Z forward. The vehicle body frame uses the same axes, so a
// vehicle with heading (yaw about +Y) 0 faces -Z, and a positive heading turns
// it to the left (towards -X). Units are SI: metres, seconds, radians, kg.
//
// Implementations must be deterministic and advanced with a fixed timestep.
class IVehicleModel
{
public:
    virtual ~IVehicleModel() = default;

    // Sets the driver controls used by subsequent step() calls.
    virtual void setInput(const VehicleInput& input) = 0;
    virtual const VehicleInput& input() const = 0;

    // Places the vehicle at rest at a world position with the given heading
    // (radians, rotation about +Y; 0 faces -Z).
    virtual void reset(const Vec3& position, Real headingRadians) = 0;

    // Advances the model by a fixed timestep in seconds. Returns false (and
    // does nothing) when deltaSeconds <= 0.
    virtual bool step(Real deltaSeconds) = 0;

    // World position of the vehicle reference point (m).
    virtual Vec3 position() const = 0;
    // Body orientation in world space (body -Z is the vehicle forward axis).
    virtual Quat orientation() const = 0;
    // Yaw about +Y in radians; 0 faces -Z, positive turns left.
    virtual Real heading() const = 0;
    // Linear velocity of the reference point in world space (m/s).
    virtual Vec3 linearVelocity() const = 0;
    // Angular velocity in world space (rad/s).
    virtual Vec3 angularVelocity() const = 0;
    // Signed speed along the forward axis (m/s, positive = moving forward).
    virtual Real forwardSpeed() const = 0;

    // Unit forward (body -Z) and right (body +X) directions in world space.
    virtual Vec3 forward() const = 0;
    virtual Vec3 right() const = 0;
};

} // namespace sim::vehicle
