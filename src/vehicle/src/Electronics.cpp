// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Electronics.hpp"

#include <algorithm>
#include <cmath>

namespace sim::vehicle {

void ElectronicControlUnit::setEnabled(bool absEnabled, bool tcsEnabled, bool escEnabled) {
    absEnabled_ = absEnabled;
    tcsEnabled_ = tcsEnabled;
    escEnabled_ = escEnabled;
}

void ElectronicControlUnit::update(const ElectronicsSensors& sensors, Real) {
    throttleScale_ = 1.0;
    modulation_ = BrakeModulation{};
    absActive_ = false;
    tcsActive_ = false;
    escActive_ = false;

    const Real speed = std::abs(sensors.forwardSpeed);

    // --- Traction control: cut the throttle when a driven wheel spins --------
    if (tcsEnabled_) {
        Real worstSlip = 0.0;
        bool anyDriven = false;
        for (std::size_t i = 0; i < kElectronicWheelCount; ++i) {
            if (!config_.traction.driven[i]) {
                continue;
            }
            anyDriven = true;
            worstSlip = std::max(worstSlip, sensors.wheelSlipRatio[i]);
        }
        if (anyDriven && worstSlip > config_.traction.targetSlip) {
            const Real excess = worstSlip - config_.traction.targetSlip;
            throttleScale_ = std::min(throttleScale_, std::clamp(1.0 - config_.traction.proportionalGain * excess,
                                                                 config_.traction.minimumScale, 1.0));
            tcsActive_ = throttleScale_ < 1.0;
        }
    }

    // --- Anti-lock brakes: release torque on a locking wheel -----------------
    if (absEnabled_ && speed >= config_.abs.minimumSpeed) {
        for (std::size_t i = 0; i < kElectronicWheelCount; ++i) {
            const Real slip = std::abs(sensors.wheelSlipRatio[i]);
            if (slip <= config_.abs.targetSlip) {
                continue;
            }
            const Real excess = slip - config_.abs.targetSlip;
            const Real fraction = std::clamp(config_.abs.proportionalGain * excess, 0.0, 1.0);
            modulation_.torqueDelta[i] -= fraction * config_.abs.maximumReleaseTorque;
            absActive_ = true;
        }
    }

    // --- Stability control: brake the outer front wheel on excess yaw --------
    const StabilityControlConfig& esc = config_.stability;
    if (escEnabled_ && speed >= config_.stability.minimumSpeed && config_.stability.wheelbase > 0.0) {
        const Real denominator = config_.stability.wheelbase + config_.stability.understeerGradient * speed * speed;
        if (denominator > 0.0) {
            const Real referenceYawRate = sensors.forwardSpeed * sensors.steerAngle / denominator;
            const Real excess = std::abs(sensors.yawRate - referenceYawRate) - config_.stability.yawRateThreshold;
            if (excess > 0.0) {
                const Real amount = std::clamp(config_.stability.proportionalGain * excess, 0.0, 1.0);
                const std::size_t outer = sensors.yawRate > referenceYawRate ? 1u : 0u; // front-right / front-left
                modulation_.torqueDelta[outer] += amount * config_.stability.maximumBrakeTorque;
                throttleScale_ = std::min(throttleScale_, 1.0 - amount);
                escActive_ = true;
            }
        }
    }
}

VehicleInput ElectronicControlUnit::filterInput(const VehicleInput& driverInput) const {
    VehicleInput filtered = driverInput;
    filtered.throttle *= throttleScale_;
    return filtered;
}

BrakeModulation ElectronicControlUnit::brakeModulation() const {
    return modulation_;
}

} // namespace sim::vehicle
