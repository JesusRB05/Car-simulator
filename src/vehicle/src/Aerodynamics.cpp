// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Aerodynamics.hpp"

#include <cmath>

#include <glm/glm.hpp>

namespace sim::vehicle {

Real airDensity(Real pressure, Real temperature, Real specificGasConstant) {
    if (temperature <= 0.0) {
        return 0.0;
    }
    return pressure / (specificGasConstant * temperature);
}

AeroLoads computeAerodynamicLoads(const Vec3& airVelocityChassis, Real density, Real dragCoefficient,
                                  Real liftCoefficient, Real frontalArea, const Vec3& centreOfPressure,
                                  const Vec3& referencePoint) {
    AeroLoads loads;
    const Real speed = glm::length(airVelocityChassis);
    if (speed <= 0.0 || density <= 0.0) {
        return loads;
    }
    const Real dynamicPressure = 0.5 * density * speed * speed; // 1/2 rho v^2

    // Drag opposes the car, so it points along the air velocity: F_D = q C_D A v_hat.
    const Vec3 drag = dynamicPressure * dragCoefficient * frontalArea * (airVelocityChassis / speed);
    // Positive C_L is downforce, i.e. -Y.
    const Vec3 lift{0.0, -dynamicPressure * liftCoefficient * frontalArea, 0.0};

    loads.force = drag + lift;
    loads.moment = glm::cross(centreOfPressure - referencePoint, loads.force);
    return loads;
}

AeroLoads AerodynamicModel::computeLoads(const Vec3& airVelocityChassis, Real airDensity) const {
    return computeAerodynamicLoads(airVelocityChassis, airDensity, dragCoefficient, liftCoefficient, frontalArea,
                                   centreOfPressure, referencePoint);
}

} // namespace sim::vehicle
