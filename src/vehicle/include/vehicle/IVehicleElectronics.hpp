// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <array>

#include "physics/PhysicsTypes.hpp"
#include "vehicle/IVehicleModel.hpp"

namespace sim::vehicle {

using physics::Real;

// Sensor snapshot available to driver aids, gathered once per fixed step.
struct ElectronicsSensors {
    std::array<Real, 4> wheelAngularVelocity{}; // rad/s, FL, FR, RL, RR (+ = rolling forward)
    std::array<Real, 4> wheelSlipRatio{};       // dimensionless
    Real yawRate = 0.0;                         // rad/s about +Y (+ = turning left)
    Real lateralAcceleration = 0.0;             // m/s^2 along body +X (right)
    Real forwardSpeed = 0.0;                    // m/s
    Real steerAngle = 0.0;                      // rad, road-wheel angle (+ = left)
};

// Per-wheel brake torque adjustments requested by ABS/ESC (N*m added to the
// base brake torque; negative values release pressure).
struct BrakeModulation {
    std::array<Real, 4> torqueDelta{};
};

// Driver-aid electronics (ABS, TCS, ESC). The hooks may modify the driver's
// request before it reaches the drivetrain and brakes. Implementations must be
// deterministic and depend only on the sensor snapshots and their own state.
class IVehicleElectronics {
public:
    virtual ~IVehicleElectronics() = default;

    virtual void setEnabled(bool absEnabled, bool tcsEnabled, bool escEnabled) = 0;

    // Updates internal state (filters, timers) with this step's sensor data.
    virtual void update(const ElectronicsSensors& sensors, Real deltaSeconds) = 0;

    // Filters the driver's input (e.g. TCS cutting throttle) and returns the
    // input the vehicle should actually use.
    virtual VehicleInput filterInput(const VehicleInput& driverInput) const = 0;

    // Per-wheel brake adjustments from ABS (release) and ESC (selective braking).
    virtual BrakeModulation brakeModulation() const = 0;
};

} // namespace sim::vehicle
