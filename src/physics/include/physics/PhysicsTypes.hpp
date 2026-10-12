// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace sim::physics {

// The physics core works in SI units with double precision world positions.
using Real = double;
using Vec3 = glm::dvec3;
using Quat = glm::dquat;
using Mat3 = glm::dmat3;

} // namespace sim::physics
