// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Driver-side drivetrain controls.
struct DrivetrainInput {
    Real throttle = 0.0; // [0, 1]
    Real clutch = 0.0;   // [0, 1], 1 = pedal fully pressed (disengaged)
    int gear = 0;        // -1 = reverse, 0 = neutral, 1..N = forward gears
};

// Power path: engine torque -> clutch -> gearbox -> differential(s) -> wheel
// torques. The drivetrain owns the rotating inertias up to the differential
// outputs; wheel spin is integrated by the vehicle/tyre coupling and fed back
// through setWheelAngularVelocity(). Wheels are indexed FL, FR, RL, RR.
class IDrivetrain {
public:
    virtual ~IDrivetrain() = default;

    virtual void setInput(const DrivetrainInput& input) = 0;

    // Feeds back the angular velocity of a wheel (rad/s, + = rolling forward).
    virtual void setWheelAngularVelocity(std::size_t wheelIndex, Real omega) = 0;

    // Advances internal state (engine speed, clutch slip) by a fixed timestep.
    virtual void step(Real deltaSeconds) = 0;

    // Drive torque delivered to a wheel hub (N*m, + = accelerating forward).
    virtual Real wheelTorque(std::size_t wheelIndex) const = 0;
    virtual std::size_t wheelCount() const = 0;

    virtual Real engineAngularVelocity() const = 0; // rad/s
    virtual Real engineTorque() const = 0;          // N*m at the crankshaft
    virtual int currentGear() const = 0;            // same encoding as DrivetrainInput::gear
};

} // namespace sim::vehicle
