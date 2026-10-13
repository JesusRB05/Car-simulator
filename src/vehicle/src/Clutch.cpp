// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Clutch.hpp"

#include <cmath>

namespace sim::vehicle {

Real FrictionClutch::clampForce() const {
    return maxClampForce * engagement;
}

Real FrictionClutch::effectiveRadius() const {
    const Real outerSquared = outerRadius * outerRadius;
    const Real innerSquared = innerRadius * innerRadius;
    const Real denominator = outerSquared - innerSquared;
    if (denominator <= 0.0) {
        return 0.0;
    }
    const Real numerator = outerRadius * outerSquared - innerRadius * innerSquared;
    return (2.0 / 3.0) * numerator / denominator;
}

Real FrictionClutch::capacity() const {
    return frictionCoefficient * clampForce() * effectiveRadius() * static_cast<Real>(frictionFaces);
}

ClutchTorque computeClutchTorque(const FrictionClutch& clutch, Real slipSpeed, Real requiredTorque) {
    const Real capacity = clutch.capacity();

    if (slipSpeed > clutch.lockThreshold || slipSpeed < -clutch.lockThreshold) {
        const Real sign = std::tanh(slipSpeed / clutch.slipSpeed);
        return {capacity * sign, true};
    }

    ClutchTorque result;
    result.torque = requiredTorque;
    if (result.torque > capacity) {
        result.torque = capacity;
        result.slipping = true;
    } else if (result.torque < -capacity) {
        result.torque = -capacity;
        result.slipping = true;
    }
    return result;
}

Real centrifugalClutchCapacity(Real coefficient, Real engineAngularVelocity, Real engageAngularVelocity) {
    const Real squared = engineAngularVelocity * engineAngularVelocity - engageAngularVelocity * engageAngularVelocity;
    return coefficient * (squared > 0.0 ? squared : 0.0);
}

} // namespace sim::vehicle
