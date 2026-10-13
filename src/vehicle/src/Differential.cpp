// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Differential.hpp"

#include <cmath>

namespace sim::vehicle {

DifferentialOutput computeDifferentialTorques(const DifferentialCoefficients& coefficients,
                                              const DifferentialInput& input) {
    DifferentialOutput output;
    const Real half = 0.5 * input.inputTorque;
    const Real delta = input.leftAngularVelocity - input.rightAngularVelocity;

    switch (coefficients.type) {
    case DifferentialType::Open: {
        output.leftTorque = half;
        output.rightTorque = half;
        break;
    }
    case DifferentialType::ClutchPack: {
        const Real ramp = (input.inputTorque >= 0.0) ? coefficients.powerRamp : coefficients.coastRamp;
        const Real capacity = coefficients.preloadTorque + ramp * std::fabs(input.inputTorque);
        const Real transfer = capacity * std::tanh(delta / coefficients.slipScale);
        output.leftTorque = half - 0.5 * transfer;
        output.rightTorque = half + 0.5 * transfer;
        break;
    }
    case DifferentialType::Torsen: {
        const Real s = std::tanh(delta / coefficients.slipScale);
        const Real bias = 1.0 + (coefficients.torqueBiasRatio - 1.0) * std::fabs(s);
        // The slower wheel receives the biased (larger) share.
        const Real slow = input.inputTorque * bias / (1.0 + bias);
        const Real fast = input.inputTorque / (1.0 + bias);
        output.leftTorque = (s >= 0.0) ? fast : slow;
        output.rightTorque = (s >= 0.0) ? slow : fast;
        break;
    }
    case DifferentialType::Viscous:
    case DifferentialType::Locked: {
        const Real coefficient = (coefficients.type == DifferentialType::Locked) ? coefficients.lockStiffness
                                                                                 : coefficients.viscousCoefficient;
        const Real transfer = coefficient * delta;
        output.leftTorque = half - 0.5 * transfer;
        output.rightTorque = half + 0.5 * transfer;
        break;
    }
    }
    return output;
}

} // namespace sim::vehicle
