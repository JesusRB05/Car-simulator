// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "render/Camera.hpp"
#include "render/Scene.hpp"
#include "render/ShaderCache.hpp"
#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"
#include "rhi/Pipeline.hpp"
#include "rhi/Resources.hpp"
#include "rhi/Types.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::render {

// Owns the scene pipeline, the per-frame uniform buffers / descriptor sets and
// the depth target. The application drives it: ensureTargets(), then
// updateFrameUniforms() and renderScene() each frame.
class Renderer
{
public:
    Renderer(rhi::Device& device, vfs::VirtualFileSystem& vfs, std::uint32_t framesInFlight);
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // (Re)creates swapchain-dependent resources when the size or format change.
    void ensureTargets(rhi::Extent2D extent, rhi::Format colorFormat);

    void updateFrameUniforms(std::uint32_t slot, const Camera& camera, const glm::vec4& lightDirection, const glm::vec4& lightColor);

    void renderScene(rhi::CommandList& commandList, const Scene& scene, const rhi::ColorAttachment& colorAttachment, std::uint32_t slot);

    bool ready() const { return pipeline_ != nullptr && depthImage_ != nullptr; }

private:
    void createPipeline(rhi::Format colorFormat);

    rhi::Device& device_;
    ShaderCache shaders_;

    std::unique_ptr<rhi::DescriptorSetLayout> frameLayout_;
    std::unique_ptr<rhi::DescriptorPool> descriptorPool_;
    std::vector<rhi::DescriptorSet> frameSets_;
    std::vector<std::unique_ptr<rhi::Buffer>> frameBuffers_;

    std::unique_ptr<rhi::GraphicsPipeline> pipeline_;
    rhi::Format colorFormat_ = rhi::Format::Undefined;

    std::unique_ptr<rhi::Image> depthImage_;
    rhi::Extent2D depthExtent_{};
};

} // namespace sim::render
