// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Scene.hpp"

#include <glm/gtc/matrix_transform.hpp>

namespace sim::render {

void Scene::add(const std::shared_ptr<Mesh>& mesh, const glm::mat4& transform, const glm::vec4& color) {
    if (mesh) {
        Drawable drawable;
        drawable.mesh = mesh;
        drawable.transform = transform;
        drawable.material.baseColor = color;
        drawables_.push_back(std::move(drawable));
    }
}

void Scene::add(const std::shared_ptr<Mesh>& mesh, const glm::mat4& transform, const Material& material) {
    if (mesh) {
        Drawable drawable;
        drawable.mesh = mesh;
        drawable.transform = transform;
        drawable.material = material;
        drawables_.push_back(std::move(drawable));
    }
}

void Scene::setDirectionalLight(const glm::vec3& direction, const glm::vec3& color) {
    lightDirection_ = glm::vec4(glm::normalize(direction), 0.0f);
    lightColor_ = glm::vec4(color, 1.0f);
}

} // namespace sim::render
