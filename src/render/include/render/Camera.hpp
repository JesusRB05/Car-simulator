// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <glm/glm.hpp>

namespace sim::render {

// A right-handed perspective camera. World space uses metres, +Y up and -Z
// forward, matching the engine conventions.
class Camera {
public:
    void setPerspective(float fovYDegrees, float aspect, float nearPlane, float farPlane);
    void lookAt(const glm::vec3& eye, const glm::vec3& target, const glm::vec3& up = {0.0f, 1.0f, 0.0f});

    const glm::mat4& view() const { return view_; }
    const glm::mat4& projection() const { return projection_; }
    glm::mat4 viewProjection() const { return projection_ * view_; }
    const glm::vec3& position() const { return position_; }

private:
    glm::mat4 view_{1.0f};
    glm::mat4 projection_{1.0f};
    glm::vec3 position_{0.0f};
};

} // namespace sim::render
