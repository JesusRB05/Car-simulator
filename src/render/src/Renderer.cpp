// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Renderer.hpp"

#include <cstddef>

#include "core/Log.hpp"
#include "render/Uniforms.hpp"

namespace sim::render {

Renderer::Renderer(rhi::Device& device, vfs::VirtualFileSystem& vfs, std::uint32_t framesInFlight)
    : device_(device), shaders_(device, vfs) {
    framesInFlight = framesInFlight < 1 ? 1 : framesInFlight;

    frameLayout_ = std::make_unique<rhi::DescriptorSetLayout>(
        device_, std::vector<rhi::DescriptorBinding>{{0, rhi::DescriptorType::UniformBuffer, 1,
                                                      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT}});

    descriptorPool_ = std::make_unique<rhi::DescriptorPool>(device_, framesInFlight, frameLayout_->bindings());

    frameSets_.reserve(framesInFlight);
    frameBuffers_.reserve(framesInFlight);
    for (std::uint32_t slot = 0; slot < framesInFlight; ++slot) {
        frameBuffers_.push_back(
            std::make_unique<rhi::Buffer>(device_, sizeof(FrameUniforms), rhi::BufferUsage::Uniform));
        rhi::DescriptorSet set(&device_, descriptorPool_->allocate(*frameLayout_));
        set.writeBuffer(0, frameBuffers_.back()->handle(), sizeof(FrameUniforms));
        frameSets_.push_back(set);
    }
}

Renderer::~Renderer() = default;

void Renderer::ensureTargets(rhi::Extent2D extent, rhi::Format colorFormat) {
    if (colorFormat != colorFormat_) {
        colorFormat_ = colorFormat;
        createPipeline(colorFormat);
    }

    if (depthImage_ == nullptr || depthExtent_.width != extent.width || depthExtent_.height != extent.height) {
        if (extent.width == 0 || extent.height == 0) {
            return;
        }
        depthImage_ = std::make_unique<rhi::Image>(device_, extent, rhi::Format::D32Sfloat,
                                                   rhi::ImageUsage::DepthStencilAttachment, true);
        depthExtent_ = extent;
    }
}

void Renderer::createPipeline(rhi::Format colorFormat) {
    const rhi::ShaderModule* vertex = shaders_.get("shaders/mesh.vertex.spv");
    const rhi::ShaderModule* fragment = shaders_.get("shaders/mesh.fragment.spv");
    if (vertex == nullptr || fragment == nullptr) {
        core::Log::get("render")->error("scene shaders are unavailable; the scene pass will be skipped");
        pipeline_.reset();
        return;
    }

    rhi::GraphicsPipelineDesc desc;
    desc.vertexBindings = {{0, sizeof(Vertex), false}};
    desc.vertexAttributes = {{0, 0, rhi::VertexFormat::Float3, offsetof(Vertex, position)},
                             {1, 0, rhi::VertexFormat::Float3, offsetof(Vertex, normal)},
                             {2, 0, rhi::VertexFormat::Float2, offsetof(Vertex, uv)}};
    desc.vertexStage = {vertex, "vertexMain"};
    desc.fragmentStage = {fragment, "fragmentMain"};
    desc.topology = rhi::PrimitiveTopology::TriangleList;
    desc.cullMode = rhi::CullMode::Back;
    desc.depthTest = true;
    desc.depthWrite = true;
    // Reversed-Z: near maps to 1 and far to 0, so a fragment passes when its
    // depth is greater-or-equal to the stored value.
    desc.depthCompare = rhi::CompareOp::GreaterEqual;
    desc.blendMode = rhi::BlendMode::Opaque;
    desc.colorFormats = {colorFormat};
    desc.depthFormat = rhi::Format::D32Sfloat;
    desc.descriptorLayouts = {frameLayout_.get()};
    desc.pushConstantSize = sizeof(ObjectPushConstants);

    pipeline_ = std::make_unique<rhi::GraphicsPipeline>(device_, desc);
    core::Log::get("render")->info("scene pipeline ready ({})", rhi::formatName(colorFormat));
}

void Renderer::updateFrameUniforms(std::uint32_t slot, const Camera& camera, const glm::vec4& lightDirection,
                                   const glm::vec4& lightColor) {
    if (slot >= frameBuffers_.size()) {
        return;
    }

    FrameUniforms uniforms{};
    const glm::mat4 viewProjection = camera.viewProjection();
    writeMatrix(viewProjection, &uniforms.viewProjection);
    uniforms.cameraPosition = glm::vec4(camera.position(), 1.0f);
    uniforms.lightDirection = lightDirection;
    uniforms.lightColor = lightColor;

    frameBuffers_[slot]->upload(&uniforms, sizeof(uniforms));
}

void Renderer::renderScene(rhi::CommandList& commandList, const Scene& scene,
                           const rhi::ColorAttachment& colorAttachment, std::uint32_t slot) {
    if (!ready() || slot >= frameSets_.size()) {
        return;
    }

    commandList.transitionImage(depthImage_->handle(), rhi::ImageLayout::Undefined, rhi::ImageLayout::DepthAttachment,
                                VK_IMAGE_ASPECT_DEPTH_BIT);

    rhi::DepthAttachment depth;
    depth.image = reinterpret_cast<std::uint64_t>(depthImage_->view());
    depth.layout = rhi::ImageLayout::DepthAttachment;
    depth.loadOp = rhi::LoadOp::Clear;
    // Cleared to the far value (0) because the projection is reversed-Z.
    depth.clearDepth = 0.0f;

    commandList.beginRendering({colorAttachment}, &depth, depthExtent_);
    commandList.bindPipeline(*pipeline_);
    commandList.bindDescriptorSet(pipeline_->layout(), 0, frameSets_[slot].handle());
    commandList.setViewport(0.0f, 0.0f, static_cast<float>(depthExtent_.width),
                            static_cast<float>(depthExtent_.height));
    commandList.setScissor(0, 0, depthExtent_.width, depthExtent_.height);

    for (const Drawable& drawable : scene.drawables()) {
        if (!drawable.mesh) {
            continue;
        }

        ObjectPushConstants push{};
        writeMatrix(drawable.transform, &push.model);
        push.baseColor = drawable.baseColor;

        commandList.pushConstants(pipeline_->layout(), VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                                  sizeof(push), &push);
        commandList.bindVertexBuffer(drawable.mesh->vertexBuffer());
        commandList.bindIndexBuffer(drawable.mesh->indexBuffer());
        commandList.drawIndexed(drawable.mesh->indexCount());
    }

    commandList.endRendering();
}

} // namespace sim::render
