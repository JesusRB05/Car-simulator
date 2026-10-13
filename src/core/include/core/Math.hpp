// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Common math aliases and constants shared by every subsystem.
//
// Unit conventions (SI internally everywhere):
//   length m, mass kg, time s, angle rad, force N, torque N*m, pressure Pa, temperature K.
// Coordinate convention: right-handed, +Y up, -Z forward for vehicles.
//
// World-space positions and transforms are stored in double precision. Local,
// per-object maths may use float where documented.

#include <numbers>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace sim::core {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kDegToRad = std::numbers::pi / 180.0;
inline constexpr double kRadToDeg = 180.0 / std::numbers::pi;
inline constexpr double kGravity = 9.80665; // m/s^2, standard gravity

// World space (double precision).
using Vec3 = glm::dvec3;
using Vec4 = glm::dvec4;
using Mat4 = glm::dmat4;
using Quat = glm::dquat;

// Local / render space (single precision). The renderer works relative to the
// camera, which keeps enough precision for very large circuits.
using Vec3f = glm::vec3;
using Vec4f = glm::vec4;
using Mat4f = glm::mat4;
using Quatf = glm::quat;

inline constexpr double kEpsilon = 1.0e-9;

} // namespace sim::core
