// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "OrbitCamera.hpp"

#include <algorithm>
#include <cmath>

#include "render/Camera.hpp"
#include "render/Scene.hpp"

namespace sim::app {
namespace {

constexpr float kMinDistance = 0.5f;
constexpr float kMaxDistance = 500.0f;
constexpr float kPitchLimit = 1.55f;
constexpr float kOrbitSpeed = 0.006f; // radians per pixel
constexpr float kNear = 0.1f;
constexpr float kFar = 1000.0f;

} // namespace

glm::vec3 OrbitCamera::eye() const
{
    const float cp = std::cos(pitch);
    return target + distance * glm::vec3(cp * std::sin(yaw), std::sin(pitch), cp * std::cos(yaw));
}

void OrbitCamera::fromCamera(const render::Camera& camera, const glm::vec3& focus)
{
    // glm::lookAt stores -forward in the third row of the view rotation.
    const glm::mat4& view = camera.view();
    const glm::vec3 forward = -glm::vec3(view[0][2], view[1][2], view[2][2]);
    const glm::vec3 position = camera.position();

    float depth = glm::dot(focus - position, forward);
    if (depth < kMinDistance)
    {
        depth = std::max(glm::length(focus - position), kMinDistance);
    }
    distance = std::clamp(depth, kMinDistance, kMaxDistance);
    target = position + forward * distance;

    const glm::vec3 offset = -forward; // from target towards the eye
    yaw = std::atan2(offset.x, offset.z);
    pitch = std::clamp(std::asin(std::clamp(offset.y, -1.0f, 1.0f)), -kPitchLimit, kPitchLimit);
}

void OrbitCamera::frame(const glm::vec3& center, float radius)
{
    const float halfFov = glm::radians(fovDegrees) * 0.5f;
    target = center;
    distance = std::clamp(std::max(radius, 0.1f) / std::sin(halfFov) * 1.1f, kMinDistance, kMaxDistance);
}

void OrbitCamera::orbit(float dx, float dy)
{
    yaw -= dx * kOrbitSpeed;
    pitch = std::clamp(pitch + dy * kOrbitSpeed, -kPitchLimit, kPitchLimit);
}

void OrbitCamera::pan(float dx, float dy, float viewportHeight)
{
    // Scale so the point under the cursor at the target depth follows it.
    const float worldPerPixel =
        2.0f * distance * std::tan(glm::radians(fovDegrees) * 0.5f) / std::max(viewportHeight, 1.0f);
    const glm::vec3 forward = glm::normalize(target - eye());
    const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    const glm::vec3 up = glm::cross(right, forward);
    target += (-right * dx + up * dy) * worldPerPixel;
}

void OrbitCamera::zoom(float wheelSteps)
{
    distance = std::clamp(distance * std::pow(0.88f, wheelSteps), kMinDistance, kMaxDistance);
}

void OrbitCamera::apply(render::Camera& camera, float aspect) const
{
    camera.setPerspective(fovDegrees, aspect, kNear, kFar);
    camera.lookAt(eye(), target, glm::vec3(0.0f, 1.0f, 0.0f));
}

bool sceneBounds(const render::Scene& scene, const render::Mesh* exclude, glm::vec3& center, float& radius)
{
    glm::vec3 lo(0.0f);
    glm::vec3 hi(0.0f);
    bool any = false;
    for (const render::Drawable& drawable : scene.drawables())
    {
        if (drawable.mesh.get() == exclude)
        {
            continue;
        }
        // Unit meshes fit in a sphere of radius ~1 around their origin, so the
        // longest basis column bounds the drawable.
        const glm::mat4& m = drawable.transform;
        const float extent = std::max({glm::length(glm::vec3(m[0])), glm::length(glm::vec3(m[1])), glm::length(glm::vec3(m[2]))});
        const glm::vec3 origin(m[3]);
        const glm::vec3 boxLo = origin - glm::vec3(extent);
        const glm::vec3 boxHi = origin + glm::vec3(extent);
        lo = any ? glm::min(lo, boxLo) : boxLo;
        hi = any ? glm::max(hi, boxHi) : boxHi;
        any = true;
    }
    if (!any)
    {
        return false;
    }
    center = (lo + hi) * 0.5f;
    radius = glm::length(hi - lo) * 0.5f;
    return true;
}

} // namespace sim::app
