// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/MassProperties.hpp"

#include <glm/glm.hpp>

namespace sim::vehicle {

MassProperties combineMassProperties(const std::vector<MassContribution>& contributions) {
    MassProperties result;
    for (const MassContribution& c : contributions) {
        result.mass += c.mass;
        result.centreOfMass += c.mass * c.position;
    }
    if (result.mass <= 0.0) {
        return result;
    }
    result.centreOfMass /= result.mass;

    for (const MassContribution& c : contributions) {
        const Vec3 r = c.position - result.centreOfMass;
        const Mat3 parallelAxis = c.mass * (glm::dot(r, r) * Mat3(1.0) - glm::outerProduct(r, r));
        result.inertia += c.inertia + parallelAxis;
    }
    return result;
}

} // namespace sim::vehicle
