// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Brake system: pedal and handbrake positions in, per-wheel brake torque
// capacity out. Wheels are indexed FL, FR, RL, RR.
class IBrakeSystem {
public:
    virtual ~IBrakeSystem() = default;

    // Pedal position and handbrake lever, both in [0, 1].
    virtual void setPedal(Real pedal) = 0;
    virtual void setHandbrake(Real handbrake) = 0;

    // Front brake bias: fraction of the total pedal brake torque sent to the
    // front axle, in [0, 1].
    virtual void setBias(Real frontBias) = 0;
    virtual Real bias() const = 0;

    // Maximum friction torque the brake can apply to a wheel (N*m, >= 0). The
    // caller applies it opposing wheel rotation and never lets it reverse spin.
    virtual Real brakeTorque(std::size_t wheelIndex) const = 0;
    virtual std::size_t wheelCount() const = 0;
};

} // namespace sim::vehicle
