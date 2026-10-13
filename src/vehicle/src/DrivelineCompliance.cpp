// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/DrivelineCompliance.hpp"

#include <algorithm>

namespace sim::vehicle {

Real computeDrivelineTorque(const DrivelineCompliance& compliance, const DrivelineState& state) {
    const Real half = 0.5 * compliance.backlash;
    const Real clamped = std::clamp(state.twist, -half, half);
    const Real effectiveTwist = state.twist - clamped;
    return compliance.stiffness * effectiveTwist + compliance.damping * state.twistRate;
}

} // namespace sim::vehicle
