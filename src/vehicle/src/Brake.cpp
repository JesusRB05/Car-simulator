// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Brake.hpp"

#include <cmath>

namespace sim::vehicle {

Real DiscBrake::torque(Real linePressure) const {
    const Real effective = linePressure - thresholdPressure;
    if (effective <= 0.0) {
        return 0.0;
    }
    return 2.0 * padFrictionCoefficient * pistonArea * effective * effectiveRadius;
}

Real frontLinePressure(Real masterPressure, const BrakeBias& bias) {
    return masterPressure * bias.frontFraction;
}

Real rearLinePressure(Real masterPressure, const BrakeBias& bias) {
    return masterPressure * (1.0 - bias.frontFraction) + bias.handbrakePressure;
}

Real stepBrakeTemperature(const BrakeThermal& thermal, Real temperature, Real brakeTorque, Real wheelAngularVelocity,
                          Real deltaSeconds) {
    const Real heatCapacity = thermal.discMass * thermal.specificHeat;
    if (heatCapacity <= 0.0) {
        return temperature;
    }

    const Real power = std::fabs(brakeTorque * wheelAngularVelocity); // W
    const Real convection =
        thermal.convectionCoefficient * thermal.coolingArea * (temperature - thermal.ambientTemperature);
    Real radiation = 0.0;
    if (thermal.emissivity > 0.0) {
        const Real stefanBoltzmann = 5.670374419e-8;
        const Real t4 = temperature * temperature * temperature * temperature;
        const Real ambient4 = thermal.ambientTemperature * thermal.ambientTemperature * thermal.ambientTemperature *
                              thermal.ambientTemperature;
        radiation = thermal.emissivity * stefanBoltzmann * thermal.coolingArea * (t4 - ambient4);
    }

    return temperature + (power - convection - radiation) / heatCapacity * deltaSeconds;
}

} // namespace sim::vehicle
