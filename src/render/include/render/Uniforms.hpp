// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <cstring>

#include <glm/glm.hpp>

namespace sim::render {

// Single matrix convention for the whole renderer: GLM is column-major and
// every shader declares its matrices column_major, so a glm::mat4 is copied to
// the GPU verbatim (writeMatrix below) and multiplied as mul(M, v), i.e. M*v.
// There is no transpose anywhere. See docs/CONVENTIONS.md.
struct FrameUniforms {
    alignas(16) glm::mat4 viewProjection;
    alignas(16) glm::mat4 lightViewProjection;
    alignas(16) glm::mat4 shadowMatrix;
    alignas(16) glm::vec4 cameraPosition;
    alignas(16) glm::vec4 lightDirection; // xyz: direction the light travels
    alignas(16) glm::vec4 lightColor;     // rgb intensity
    alignas(16) glm::vec4 ambientSky;     // rgb sky ambient, a exposure
    alignas(16) glm::vec4 ambientGround;  // rgb ground ambient
};

// Per-drawable data. Kept inside the guaranteed 128-byte push-constant budget
// (64 + 3 * 16 = 112).
struct ObjectPushConstants {
    alignas(16) glm::mat4 model;
    alignas(16) glm::vec4 baseColor; // rgb albedo, a opacity
    alignas(16) glm::vec4 emissive;  // rgb emissive, a strength
    alignas(16) glm::vec4 params;    // x metallic, y roughness, z clearcoat, w alphaCutoff
};

// Writes a GLM matrix verbatim into a shader structure. GLM stores matrices
// column-major and the shaders declare column_major, so this is a straight
// copy: bytes[0..15] are column 0, bytes[16..31] column 1, etc. This is the
// single place a CPU matrix maps to the GPU upload layout.
inline void writeMatrix(const glm::mat4& matrix, void* destination) {
    std::memcpy(destination, &matrix, sizeof(matrix));
}

} // namespace sim::render
