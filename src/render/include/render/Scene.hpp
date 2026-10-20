// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "render/Camera.hpp"
#include "render/Mesh.hpp"

namespace sim::render {

class Texture;

// A physically-based material. The PBR factors are what the shaders consume; the
// two texture handles carry the content's colour and emissive maps. Both are
// optional: the renderer substitutes a white 1x1 for the base colour and a black
// 1x1 for the emissive, so the shader can always sample. `alphaBlend` selects the
// sorted transparent pass; `alphaMask` discards on `alphaCutoff` in the opaque
// pass.
struct Material {
    glm::vec4 baseColor{0.8f, 0.8f, 0.8f, 1.0f};
    glm::vec3 emissive{0.0f};
    float emissiveStrength = 1.0f;
    float metallic = 0.0f;
    float roughness = 0.5f;
    float clearcoat = 0.0f;
    float clearcoatRoughness = 0.03f;
    float alphaCutoff = 0.5f;
    bool doubleSided = false;
    bool alphaBlend = false;
    bool alphaMask = false;
    std::shared_ptr<Texture> baseColorTexture;
    std::shared_ptr<Texture> emissiveTexture;
};

struct Drawable {
    std::shared_ptr<Mesh> mesh;
    glm::mat4 transform{1.0f};
    Material material{};
};

// A flat, renderer-facing description of what should be drawn. The simulation
// fills this each frame from the interpolated physics snapshot.
class Scene {
public:
    void clear() { drawables_.clear(); }
    // Convenience overload for the many untextured demo/primitive call sites.
    void add(const std::shared_ptr<Mesh>& mesh, const glm::mat4& transform, const glm::vec4& color);
    void add(const std::shared_ptr<Mesh>& mesh, const glm::mat4& transform, const Material& material);

    std::vector<Drawable>& drawables() { return drawables_; }
    const std::vector<Drawable>& drawables() const { return drawables_; }

    Camera& camera() { return camera_; }
    const Camera& camera() const { return camera_; }

    void setDirectionalLight(const glm::vec3& direction, const glm::vec3& color);
    const glm::vec4& lightDirection() const { return lightDirection_; }
    const glm::vec4& lightColor() const { return lightColor_; }

    // Background (clear) colour. It doubles as the sky: the orientation checks
    // expect the top of a horizontal view to be this colour, and the renderer
    // derives its hemispheric ambient fill from it.
    void setSkyColor(const glm::vec4& color) { skyColor_ = color; }
    const glm::vec4& skyColor() const { return skyColor_; }

    // Exposure applied before the ACES tone map (1 = neutral).
    void setExposure(float exposure) { exposure_ = exposure; }
    float exposure() const { return exposure_; }

private:
    std::vector<Drawable> drawables_;
    Camera camera_;
    glm::vec4 skyColor_{0.42f, 0.58f, 0.80f, 1.0f};
    glm::vec4 lightDirection_{-0.4f, -1.0f, -0.3f, 0.0f};
    glm::vec4 lightColor_{1.0f, 1.0f, 1.0f, 1.0f};
    float exposure_ = 1.0f;
};

} // namespace sim::render
