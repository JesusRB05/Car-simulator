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

} // namespace sim::vehicle
