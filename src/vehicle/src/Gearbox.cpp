// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Gearbox.hpp"

#include <cstddef>

namespace sim::vehicle {

Real Gearbox::ratio(int gear) const {
    if (gear == 0) {
        return 0.0;
    }
    if (gear < 0) {
        return -reverseRatio;
    }
    const std::size_t index = static_cast<std::size_t>(gear - 1);
    if (index >= forwardRatios.size()) {
        return 0.0;
    }
    return forwardRatios[index];
}

Real Gearbox::outputTorque(int gear, Real inputTorque) const {
    return ratio(gear) * efficiency * inputTorque;
}

Real Gearbox::reflectedInertia(int gear, Real inputInertia, Real outputInertia) const {
    const Real g = ratio(gear);
    return outputInertia + g * g * inputInertia;
}

Real TorqueConverter::pumpTorque(Real pumpAngularVelocity) const {
    return capacityFactor * pumpAngularVelocity * pumpAngularVelocity;
}

Real TorqueConverter::turbineTorque(Real pumpTorque) const {
    return torqueRatio * pumpTorque;
}

void SequentialShift::request(int gear) {
    if (active_ || gear == currentGear_) {
        return;
    }
    targetGear_ = gear;
    elapsed_ = 0.0;
    active_ = true;
}

void SequentialShift::step(Real deltaSeconds) {
    if (!active_) {
        return;
    }
    elapsed_ += deltaSeconds;
    if (elapsed_ >= shiftTime) {
        currentGear_ = targetGear_; // the new gear engages when the interruption ends
    }
    if (elapsed_ >= shiftTime + blendTime - 1.0e-9) {
        active_ = false;
    }
}

Real SequentialShift::torqueFactor() const {
    if (!active_) {
        return 1.0;
    }
    if (elapsed_ < shiftTime) {
        return 0.0;
    }
    if (blendTime <= 0.0) {
        return 1.0;
    }
    const Real f = (elapsed_ - shiftTime) / blendTime;
    return (f < 1.0) ? f : 1.0;
}

} // namespace sim::vehicle
