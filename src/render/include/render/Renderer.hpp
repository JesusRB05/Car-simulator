// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "render/Camera.hpp"
#include "render/Scene.hpp"
#include "render/ShaderCache.hpp"
#include "render/Texture.hpp"
#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"
#include "rhi/Pipeline.hpp"
#include "rhi/Resources.hpp"
#include "rhi/Types.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::render {

// Owns the scene pipelines (opaque + transparent), the directional shadow map,
// the per-frame uniform buffers / descriptor sets and the depth target. The
// application drives it: ensureTargets(), then updateFrameUniforms() and
// renderScene() each frame.
class Renderer {
public:
    Renderer(rhi::Device& device, vfs::VirtualFileSystem& vfs, std::uint32_t framesInFlight);
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // (Re)creates swapchain-dependent resources when the size or format change.
    void ensureTargets(rhi::Extent2D extent, rhi::Format colorFormat);

    // Computes the frame, light and shadow matrices for `slot` from the scene.
    void updateFrameUniforms(std::uint32_t slot, const Scene& scene);

    void renderScene(rhi::CommandList& commandList, const Scene& scene, const rhi::ColorAttachment& colorAttachment,
                     std::uint32_t slot);

    // Drops every cached per-material texture set and returns the descriptor sets
    // to the pool. Call it when a scene and the textures it owned are destroyed,
    // before the image views could be recycled.
    void releaseMaterialSets();

    bool ready() const { return pipeline_ != nullptr && depthImage_ != nullptr; }

private:
    void ensurePipelines(rhi::Format colorFormat);
    void ensureShadowMaps();
    // The descriptor set for a drawable's two material maps, created on first
    // use and reused afterwards. Untextured materials share the defaults.
    const rhi::DescriptorSet& materialSet(const Material& material);

    rhi::Device& device_;
    ShaderCache shaders_;

    std::unique_ptr<rhi::DescriptorSetLayout> frameLayout_;
    std::unique_ptr<rhi::DescriptorPool> descriptorPool_;
    std::vector<rhi::DescriptorSet> frameSets_;
    std::vector<std::unique_ptr<rhi::Buffer>> frameBuffers_;

    // Set 1: the directional shadow map (one per frame in flight, so a frame's
    // shadow pass never races another frame's shading).
    std::unique_ptr<rhi::DescriptorSetLayout> shadowLayout_;
    std::unique_ptr<rhi::DescriptorPool> shadowPool_;
    std::unique_ptr<rhi::Sampler> shadowSampler_;
    std::vector<rhi::DescriptorSet> shadowSets_;
    std::vector<std::unique_ptr<rhi::Image>> shadowImages_;

    // Set 2: the material maps. One 1x1 white and one 1x1 black texture stand in
    // for a material that carries none, so the shader samples unconditionally;
    // every distinct pair of views then gets one descriptor set, cached here.
    std::unique_ptr<rhi::DescriptorSetLayout> materialLayout_;
    std::unique_ptr<rhi::DescriptorPool> materialPool_;
    std::unique_ptr<Texture> whitePixel_;
    std::unique_ptr<Texture> blackPixel_;
    std::map<std::pair<VkImageView, VkImageView>, rhi::DescriptorSet> materialSets_;

    std::unique_ptr<rhi::GraphicsPipeline> pipeline_;
    std::unique_ptr<rhi::GraphicsPipeline> transparentPipeline_;
    std::unique_ptr<rhi::GraphicsPipeline> shadowPipeline_;
    rhi::Format colorFormat_ = rhi::Format::Undefined;

    std::unique_ptr<rhi::Image> depthImage_;
    rhi::Extent2D depthExtent_{};

    // Recomputed every frame from the camera and the scene's light.
    glm::mat4 lightViewProjection_{1.0f};
};

} // namespace sim::render
