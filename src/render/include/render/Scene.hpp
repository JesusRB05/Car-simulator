// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "render/Camera.hpp"
#include "render/Mesh.hpp"

namespace sim::render {

struct Drawable
{
    std::shared_ptr<Mesh> mesh;
    glm::mat4 transform{1.0f};
    glm::vec4 baseColor{0.8f, 0.8f, 0.8f, 1.0f};
};

// A flat, renderer-facing description of what should be drawn. The simulation
// fills this each frame from the interpolated physics snapshot.
class Scene
{
public:
    void clear() { drawables_.clear(); }
    void add(const std::shared_ptr<Mesh>& mesh, const glm::mat4& transform, const glm::vec4& color);

    std::vector<Drawable>& drawables() { return drawables_; }
    const std::vector<Drawable>& drawables() const { return drawables_; }

    Camera& camera() { return camera_; }
    const Camera& camera() const { return camera_; }

    void setDirectionalLight(const glm::vec3& direction, const glm::vec3& color);
    const glm::vec4& lightDirection() const { return lightDirection_; }
    const glm::vec4& lightColor() const { return lightColor_; }

private:
    std::vector<Drawable> drawables_;
    Camera camera_;
    glm::vec4 lightDirection_{-0.4f, -1.0f, -0.3f, 0.0f};
    glm::vec4 lightColor_{1.0f, 1.0f, 1.0f, 1.0f};
};

} // namespace sim::render
