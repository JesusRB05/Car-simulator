// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <glm/glm.hpp>

namespace sim::render {

// Matrices are passed as explicit rows (float4 each) and multiplied with dot
// products inside the shader. This keeps the CPU (GLM, column-major) and the
// GPU (Slang) layouts unambiguous regardless of the matrix layout default.
struct FrameUniforms
{
    alignas(16) glm::vec4 viewProjRows[4];
    alignas(16) glm::vec4 cameraPosition;
    alignas(16) glm::vec4 lightDirection;
    alignas(16) glm::vec4 lightColor;
};

struct ObjectPushConstants
{
    alignas(16) glm::vec4 modelRows[4];
    alignas(16) glm::vec4 baseColor;
};

inline glm::vec4 matrixRow(const glm::mat4& matrix, int row)
{
    return {matrix[0][row], matrix[1][row], matrix[2][row], matrix[3][row]};
}

} // namespace sim::render
