// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <array>
#include <cstddef>

#include "physics/PhysicsTypes.hpp"
#include "vehicle/IVehicleElectronics.hpp"

namespace sim::vehicle {

using physics::Real;

// Number of wheels the aids act on and the corner order of every per-wheel
// array here and in ElectronicsSensors (front-left, front-right, rear-left,
// rear-right).
inline constexpr std::size_t kElectronicWheelCount = 4;

// One combined driver-aid unit (ABS, TCS, ESC) implementing IVehicleElectronics
// (PHYSICS_PLAN Section 7.8). The vehicle feeds it a sensor snapshot once per
// fixed step; the unit filters the driver input (TCS/ESC cutting the throttle)
// and returns per-wheel brake-torque adjustments (ABS releasing pressure, ESC
// braking one wheel). Corner order is front-left, front-right, rear-left,
// rear-right.

// Traction control: trims the throttle proportionally when a driven wheel spins
// past the target slip, so a launch or a corner exit does not turn into
// wheelspin.
struct TractionControlConfig {
    Real targetSlip = 0.12;                               // kappa at the grip peak
    Real proportionalGain = 3.0;                          // throttle-scale reduction per unit excess slip
    Real minimumScale = 0.0;                              // floor of the throttle scale
    std::array<bool, 4> driven{false, false, true, true}; // rear-wheel drive
};

// Anti-lock brakes: releases brake torque proportionally when a wheel's slip
// magnitude passes the target, corner by corner, so a wheel does not lock.
struct AbsConfig {
    Real targetSlip = 0.12;             // kappa magnitude at the grip peak
    Real proportionalGain = 4.0;        // release fraction per unit excess slip
    Real minimumSpeed = 2.0;            // m/s below this the ABS disengages
    Real maximumReleaseTorque = 3000.0; // N*m released at full correction
};

// Electronic stability control: compares the measured yaw rate against the
// neutral-steer reference v * delta / (L + K v^2); when the car yaws more than
// the driver asked for it brakes the outer front wheel and trims the throttle.
struct StabilityControlConfig {
    Real targetSlip = 0.10;           // unused now; reserved for a slip-aware blend
    Real proportionalGain = 2.0;      // correction fraction per unit yaw excess
    Real minimumSpeed = 5.0;          // m/s below this the ESC disengages
    Real yawRateThreshold = 0.02;     // rad/s of dead band before it reacts
    Real wheelbase = 2.6;             // m, L of the reference model
    Real understeerGradient = 0.0;    // K (s^2/m) of the reference model
    Real maximumBrakeTorque = 2000.0; // N*m added to the outer front wheel
};

// Combined configuration for the three aids.
struct ElectronicControlConfig {
    TractionControlConfig traction;
    AbsConfig abs;
    StabilityControlConfig stability;
};

class ElectronicControlUnit final : public IVehicleElectronics {
public:
    explicit ElectronicControlUnit(const ElectronicControlConfig& config = {}) : config_(config) {}

    // --- IVehicleElectronics ---------------------------------------------
    void setEnabled(bool absEnabled, bool tcsEnabled, bool escEnabled) override;
    void update(const ElectronicsSensors& sensors, Real deltaSeconds) override;
    VehicleInput filterInput(const VehicleInput& driverInput) const override;
    BrakeModulation brakeModulation() const override;

    // --- Diagnostics ------------------------------------------------------
    bool absActive() const { return absActive_; }
    bool tcsActive() const { return tcsActive_; }
    bool escActive() const { return escActive_; }
    Real throttleScale() const { return throttleScale_; } // [0, 1] after TCS/ESC
    const ElectronicControlConfig& config() const { return config_; }

private:
    ElectronicControlConfig config_;
    bool absEnabled_ = false;
    bool tcsEnabled_ = false;
    bool escEnabled_ = false;

    Real throttleScale_ = 1.0;
    BrakeModulation modulation_{};
    bool absActive_ = false;
    bool tcsActive_ = false;
    bool escActive_ = false;
};

} // namespace sim::vehicle
