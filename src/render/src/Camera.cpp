// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Camera.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace sim::render {

void Camera::setPerspective(float fovYDegrees, float aspect, float nearPlane, float farPlane)
{
    // Reversed-Z projection for Vulkan's [0, 1] clip depth. Mapping the near
    // plane to depth 1 and the far plane to depth 0 (instead of the usual
    // 0/1) distributes floating-point precision near the camera, which is
    // what keeps far geometry stable during this phase's depth work. The
    // matching pipeline state is CompareOp::GreaterEqual with a cleared depth
    // of 0; see Renderer::createPipeline.
    const float f = 1.0f / std::tan(glm::radians(fovYDegrees) * 0.5f);
    const float nf = 1.0f / (farPlane - nearPlane);

    projection_ = glm::mat4(0.0f);
    projection_[0][0] = f / aspect;
    projection_[1][1] = f;
    projection_[2][2] = nearPlane * nf;
    projection_[3][2] = farPlane * nearPlane * nf;
    projection_[2][3] = -1.0f;
}

void Camera::lookAt(const glm::vec3& eye, const glm::vec3& target, const glm::vec3& up)
{
    position_ = eye;
    // glm::lookAt builds a right-handed view matrix with -Z forward.
    view_ = glm::lookAt(eye, target, up);
}

} // namespace sim::render
