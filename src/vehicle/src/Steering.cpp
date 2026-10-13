// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Steering.hpp"

#include <cmath>

namespace sim::vehicle {

Real ackermannOuterAngle(const AckermannGeometry& geometry, Real innerAngle) {
    const Real cotInner = 1.0 / std::tan(innerAngle);
    const Real cotOuter = cotInner + geometry.track / geometry.wheelbase;
    return std::atan(1.0 / cotOuter);
}

SteerAngles steerAngles(const AckermannGeometry& geometry, Real innerAngle, Real ackermannPercent) {
    SteerAngles angles;
    angles.inner = innerAngle;
    const Real ideal = ackermannOuterAngle(geometry, innerAngle);
    angles.outer = innerAngle + ackermannPercent * (ideal - innerAngle);
    return angles;
}

Real roadWheelAngle(Real steeringRatio, Real steeringWheelAngle) {
    if (steeringRatio == 0.0) {
        return 0.0;
    }
    return steeringWheelAngle / steeringRatio;
}

Real steeringColumnTorque(const std::vector<WheelSteerLoad>& wheels, Real ratioEfficiency, Real assistTorque,
                          Real frictionTorque) {
    Real aligning = 0.0;
    for (const WheelSteerLoad& w : wheels) {
        aligning += w.aligningMoment + w.lateralForce * (w.mechanicalTrail + w.pneumaticTrail);
    }
    const Real efficiency = (ratioEfficiency != 0.0) ? ratioEfficiency : 1.0;
    return aligning / efficiency + assistTorque + frictionTorque;
}

} // namespace sim::vehicle
