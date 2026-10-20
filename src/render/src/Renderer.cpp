// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <glm/gtc/matrix_transform.hpp>

#include "core/Log.hpp"
#include "render/Frustum.hpp"
#include "render/Uniforms.hpp"

namespace sim::render {
namespace {

// The directional shadow map is a fixed square; 2048^2 is a good trade between
// coverage and memory for a single sun.
constexpr std::uint32_t kShadowSize = 2048;

// Ceiling on distinct (base colour, emissive) texture pairs. Each car or track
// material contributes one; a whole circuit plus a field of cars fits easily.
constexpr std::uint32_t kMaxMaterialSets = 1024;

// Reversed-Z orthographic projection for Vulkan's [0, 1] clip depth: the near
// plane maps to depth 1 and the far plane to 0, matching Camera::setPerspective
// and the GreaterEqual depth test.
glm::mat4 reversedZOrtho(float halfExtent, float nearPlane, float farPlane) {
    glm::mat4 projection(0.0f);
    projection[0][0] = 1.0f / halfExtent;
    projection[1][1] = 1.0f / halfExtent;
    projection[2][2] = 1.0f / (farPlane - nearPlane);
    projection[3][2] = farPlane / (farPlane - nearPlane);
    projection[3][3] = 1.0f;
    return projection;
}

ObjectPushConstants makePushConstants(const Drawable& drawable) {
    ObjectPushConstants push{};
    writeMatrix(drawable.transform, &push.model);
    push.baseColor = drawable.material.baseColor;
    push.emissive = glm::vec4(drawable.material.emissive, drawable.material.emissiveStrength);
    push.params = glm::vec4(drawable.material.metallic, drawable.material.roughness, drawable.material.clearcoat,
                            drawable.material.alphaMask ? drawable.material.alphaCutoff : 0.0f);
    return push;
}

} // namespace

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

    // Set 1: the shadow map, one image per frame in flight. A separate texture
    // and sampler pair, matching the shader's two bindings.
    shadowLayout_ = std::make_unique<rhi::DescriptorSetLayout>(
        device_,
        std::vector<rhi::DescriptorBinding>{{0, rhi::DescriptorType::SampledImage, 1, VK_SHADER_STAGE_FRAGMENT_BIT},
                                            {1, rhi::DescriptorType::Sampler, 1, VK_SHADER_STAGE_FRAGMENT_BIT}});
    shadowPool_ = std::make_unique<rhi::DescriptorPool>(device_, framesInFlight, shadowLayout_->bindings());
    shadowSampler_ = std::make_unique<rhi::Sampler>(device_, false, false); // nearest, clamp to edge

    shadowImages_.reserve(framesInFlight);
    shadowSets_.reserve(framesInFlight);
    for (std::uint32_t slot = 0; slot < framesInFlight; ++slot) {
        auto image =
            std::make_unique<rhi::Image>(device_, rhi::Extent2D{kShadowSize, kShadowSize}, rhi::Format::D32Sfloat,
                                         rhi::ImageUsage::DepthStencilAttachment | rhi::ImageUsage::Sampled, true);
        rhi::DescriptorSet set(&device_, shadowPool_->allocate(*shadowLayout_));
        set.writeSampledImage(0, image->view());
        set.writeSampler(1, shadowSampler_->handle());
        shadowImages_.push_back(std::move(image));
        shadowSets_.push_back(set);
    }

    // Set 2: the two material maps. The stand-ins are 1x1 so an untextured
    // material costs one texel and no branch in the shader.
    materialLayout_ = std::make_unique<rhi::DescriptorSetLayout>(
        device_, std::vector<rhi::DescriptorBinding>{
                    {0, rhi::DescriptorType::CombinedImageSampler, 1, VK_SHADER_STAGE_FRAGMENT_BIT},
                    {1, rhi::DescriptorType::CombinedImageSampler, 1, VK_SHADER_STAGE_FRAGMENT_BIT}});
    materialPool_ = std::make_unique<rhi::DescriptorPool>(device_, kMaxMaterialSets, materialLayout_->bindings());
    const std::uint8_t white[4] = {255, 255, 255, 255};
    const std::uint8_t black[4] = {0, 0, 0, 255};
    whitePixel_ = std::make_unique<Texture>(device_, 1, 1, white, /*srgb=*/false, /*repeat=*/false);
    blackPixel_ = std::make_unique<Texture>(device_, 1, 1, black, /*srgb=*/false, /*repeat=*/false);
}

Renderer::~Renderer() = default;

void Renderer::ensureTargets(rhi::Extent2D extent, rhi::Format colorFormat) {
    if (colorFormat != colorFormat_) {
        colorFormat_ = colorFormat;
        ensurePipelines(colorFormat);
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

void Renderer::ensurePipelines(rhi::Format colorFormat) {
    const rhi::ShaderModule* vertex = shaders_.get("shaders/mesh.vertex.spv");
    const rhi::ShaderModule* fragment = shaders_.get("shaders/mesh.fragment.spv");
    if (vertex == nullptr || fragment == nullptr) {
        core::Log::get("render")->error("scene shaders are unavailable; the scene pass will be skipped");
        pipeline_.reset();
        transparentPipeline_.reset();
        return;
    }

    const std::vector<rhi::VertexAttribute> attributes = {{0, 0, rhi::VertexFormat::Float3, offsetof(Vertex, position)},
                                                          {1, 0, rhi::VertexFormat::Float3, offsetof(Vertex, normal)},
                                                          {2, 0, rhi::VertexFormat::Float2, offsetof(Vertex, uv)},
                                                          {3, 0, rhi::VertexFormat::Float4, offsetof(Vertex, tangent)},
                                                          {4, 0, rhi::VertexFormat::Float4, offsetof(Vertex, color)}};

    // Reversed-Z: near maps to 1 and far to 0, so a fragment passes when its
    // depth is greater-or-equal to the stored value.
    rhi::GraphicsPipelineDesc opaque;
    opaque.vertexBindings = {{0, sizeof(Vertex), false}};
    opaque.vertexAttributes = attributes;
    opaque.vertexStage = {vertex, "vertexMain"};
    opaque.fragmentStage = {fragment, "fragmentMain"};
    opaque.topology = rhi::PrimitiveTopology::TriangleList;
    // Both faces are drawn so thin (glass) and open geometry reads correctly;
    // the shader flips the normal for back faces.
    opaque.cullMode = rhi::CullMode::None;
    opaque.depthTest = true;
    opaque.depthWrite = true;
    opaque.depthCompare = rhi::CompareOp::GreaterEqual;
    opaque.blendMode = rhi::BlendMode::Opaque;
    opaque.colorFormats = {colorFormat};
    opaque.depthFormat = rhi::Format::D32Sfloat;
    opaque.descriptorLayouts = {frameLayout_.get(), shadowLayout_.get(), materialLayout_.get()};
    opaque.pushConstantSize = sizeof(ObjectPushConstants);
    pipeline_ = std::make_unique<rhi::GraphicsPipeline>(device_, opaque);

    rhi::GraphicsPipelineDesc transparent = opaque;
    transparent.blendMode = rhi::BlendMode::AlphaBlend;
    transparent.depthWrite = false;
    transparentPipeline_ = std::make_unique<rhi::GraphicsPipeline>(device_, transparent);

    const rhi::ShaderModule* shadowVertex = shaders_.get("shaders/shadow.vertex.spv");
    const rhi::ShaderModule* shadowFragment = shaders_.get("shaders/shadow.fragment.spv");
    if (shadowVertex != nullptr && shadowFragment != nullptr) {
        rhi::GraphicsPipelineDesc shadow;
        shadow.vertexBindings = {{0, sizeof(Vertex), false}};
        shadow.vertexAttributes = {{0, 0, rhi::VertexFormat::Float3, offsetof(Vertex, position)}};
        shadow.vertexStage = {shadowVertex, "vertexMain"};
        shadow.fragmentStage = {shadowFragment, "fragmentMain"};
        shadow.topology = rhi::PrimitiveTopology::TriangleList;
        shadow.cullMode = rhi::CullMode::None;
        shadow.depthTest = true;
        shadow.depthWrite = true;
        shadow.depthCompare = rhi::CompareOp::GreaterEqual;
        shadow.blendMode = rhi::BlendMode::Opaque;
        shadow.colorFormats = {};
        shadow.depthFormat = rhi::Format::D32Sfloat;
        shadow.descriptorLayouts = {frameLayout_.get()};
        shadow.pushConstantSize = sizeof(ObjectPushConstants);
        shadowPipeline_ = std::make_unique<rhi::GraphicsPipeline>(device_, shadow);
    } else {
        core::Log::get("render")->warn("shadow shaders are unavailable; the sun will not cast shadows");
        shadowPipeline_.reset();
    }

    core::Log::get("render")->info("scene pipelines ready ({})", rhi::formatName(colorFormat));
}

void Renderer::updateFrameUniforms(std::uint32_t slot, const Scene& scene) {
    if (slot >= frameBuffers_.size()) {
        return;
    }

    FrameUniforms uniforms{};
    writeMatrix(scene.camera().viewProjection(), &uniforms.viewProjection);

    // Fit the shadow map to the region around where the camera looks. A 190 m
    // box around a point 55 m ahead keeps the circuit's kerbs and barriers
    // sharp while the sun is still overhead.
    glm::vec3 lightDirection = glm::vec3(scene.lightDirection());
    if (glm::length(lightDirection) < 1.0e-6f) {
        lightDirection = glm::vec3(0.0f, -1.0f, 0.0f);
    }
    lightDirection = glm::normalize(lightDirection);
    const glm::vec3 focus = scene.camera().position() + scene.camera().forward() * 55.0f;
    const glm::vec3 eye = focus - lightDirection * 240.0f;
    const glm::mat4 lightView = glm::lookAt(eye, focus, glm::vec3(0.0f, 1.0f, 0.0f));
    lightViewProjection_ = reversedZOrtho(95.0f, 1.0f, 640.0f) * lightView;
    writeMatrix(lightViewProjection_, &uniforms.lightViewProjection);
    writeMatrix(lightViewProjection_, &uniforms.shadowMatrix);

    uniforms.cameraPosition = glm::vec4(scene.camera().position(), 1.0f);
    uniforms.lightDirection = scene.lightDirection();
    uniforms.lightColor = scene.lightColor();

    const glm::vec3 sky(scene.skyColor());
    // A hemispheric ambient derived from the sky's luminance: bright from above,
    // darker from below. It is deliberately neutral (no sky/ground hue) so a lit
    // surface's hue stays its albedo's, which is what the orientation harness's
    // chromaticity probes assume. A tinted image-based ambient is a later step.
    const float skyLuminance = glm::dot(sky, glm::vec3(0.2126f, 0.7152f, 0.0722f));
    uniforms.ambientSky = glm::vec4(glm::vec3(skyLuminance * 0.55f), scene.exposure());
    uniforms.ambientGround = glm::vec4(glm::vec3(skyLuminance * 0.18f), 1.0f);

    frameBuffers_[slot]->upload(&uniforms, sizeof(uniforms));
}

void Renderer::releaseMaterialSets() {
    materialSets_.clear();
    if (materialPool_ != nullptr) {
        materialPool_->reset();
    }
}

const rhi::DescriptorSet& Renderer::materialSet(const Material& material) {
    const VkImageView base =
        material.baseColorTexture != nullptr ? material.baseColorTexture->view() : whitePixel_->view();
    const VkImageView emissive =
        material.emissiveTexture != nullptr ? material.emissiveTexture->view() : blackPixel_->view();
    const std::pair<VkImageView, VkImageView> key{base, emissive};

    const auto existing = materialSets_.find(key);
    if (existing != materialSets_.end()) {
        return existing->second;
    }
    if (materialSets_.size() >= kMaxMaterialSets) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            core::Log::get("render")->warn("material texture sets exceeded {}; reusing the first", kMaxMaterialSets);
        }
        return materialSets_.begin()->second;
    }

    rhi::DescriptorSet set(&device_, materialPool_->allocate(*materialLayout_));
    set.writeImage(0, material.baseColorTexture != nullptr ? material.baseColorTexture->sampler()
                                                          : whitePixel_->sampler(),
                   base);
    set.writeImage(1, material.emissiveTexture != nullptr ? material.emissiveTexture->sampler()
                                                         : blackPixel_->sampler(),
                   emissive);
    return materialSets_.emplace(key, set).first->second;
}

void Renderer::renderScene(rhi::CommandList& commandList, const Scene& scene,
                           const rhi::ColorAttachment& colorAttachment, std::uint32_t slot) {
    if (!ready() || slot >= frameSets_.size()) {
        return;
    }

    // ---------------------------------------------------------------- shadow
    const rhi::Extent2D shadowExtent{kShadowSize, kShadowSize};
    commandList.transitionImage(shadowImages_[slot]->handle(), rhi::ImageLayout::Undefined,
                                rhi::ImageLayout::DepthAttachment, VK_IMAGE_ASPECT_DEPTH_BIT);

    rhi::DepthAttachment shadowDepth;
    shadowDepth.image = reinterpret_cast<std::uint64_t>(shadowImages_[slot]->view());
    shadowDepth.layout = rhi::ImageLayout::DepthAttachment;
    shadowDepth.loadOp = rhi::LoadOp::Clear;
    // Cleared to the far value (0) because the projection is reversed-Z.
    shadowDepth.clearDepth = 0.0f;
    // The map is sampled by the scene pass, so its depth must be stored.
    shadowDepth.store = true;

    commandList.beginRendering({}, &shadowDepth, shadowExtent);
    if (shadowPipeline_ != nullptr) {
        commandList.bindPipeline(*shadowPipeline_);
        commandList.bindDescriptorSet(shadowPipeline_->layout(), 0, frameSets_[slot].handle());
        commandList.setViewport(0.0f, 0.0f, static_cast<float>(shadowExtent.width),
                                static_cast<float>(shadowExtent.height));
        commandList.setScissor(0, 0, shadowExtent.width, shadowExtent.height);

        const Frustum lightFrustum(lightViewProjection_);
        for (const Drawable& drawable : scene.drawables()) {
            if (!drawable.mesh || drawable.material.alphaBlend) {
                continue;
            }
            const Aabb world =
                transformBounds(Aabb{drawable.mesh->boundsMin(), drawable.mesh->boundsMax()}, drawable.transform);
            if (!lightFrustum.contains(world)) {
                continue;
            }
            const ObjectPushConstants push = makePushConstants(drawable);
            commandList.pushConstants(shadowPipeline_->layout(),
                                      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push),
                                      &push);
            commandList.bindVertexBuffer(drawable.mesh->vertexBuffer());
            commandList.bindIndexBuffer(drawable.mesh->indexBuffer());
            commandList.drawIndexed(drawable.mesh->indexCount());
        }
    }
    commandList.endRendering();
    commandList.transitionImage(shadowImages_[slot]->handle(), rhi::ImageLayout::DepthAttachment,
                                rhi::ImageLayout::ShaderReadOnly, VK_IMAGE_ASPECT_DEPTH_BIT);

    // ------------------------------------------------------------- main pass
    commandList.transitionImage(depthImage_->handle(), rhi::ImageLayout::Undefined, rhi::ImageLayout::DepthAttachment,
                                VK_IMAGE_ASPECT_DEPTH_BIT);

    rhi::DepthAttachment depth;
    depth.image = reinterpret_cast<std::uint64_t>(depthImage_->view());
    depth.layout = rhi::ImageLayout::DepthAttachment;
    depth.loadOp = rhi::LoadOp::Clear;
    depth.clearDepth = 0.0f;
    depth.store = false; // nothing samples the scene depth after the pass

    commandList.beginRendering({colorAttachment}, &depth, depthExtent_);

    // Everything the camera cannot see is skipped before any command is built;
    // the volume comes from the same view-projection the shaders use, so a
    // conservative test cannot disagree with what would actually rasterise.
    const Frustum frustum(scene.camera().viewProjection());

    commandList.bindPipeline(*pipeline_);
    commandList.bindDescriptorSet(pipeline_->layout(), 0, frameSets_[slot].handle());
    commandList.bindDescriptorSet(pipeline_->layout(), 1, shadowSets_[slot].handle());
    commandList.setViewport(0.0f, 0.0f, static_cast<float>(depthExtent_.width),
                            static_cast<float>(depthExtent_.height));
    commandList.setScissor(0, 0, depthExtent_.width, depthExtent_.height);

    std::vector<const Drawable*> transparent;
    for (const Drawable& drawable : scene.drawables()) {
        if (!drawable.mesh) {
            continue;
        }
        const Aabb world =
            transformBounds(Aabb{drawable.mesh->boundsMin(), drawable.mesh->boundsMax()}, drawable.transform);
        if (!frustum.contains(world)) {
            continue;
        }
        if (drawable.material.alphaBlend) {
            transparent.push_back(&drawable);
            continue;
        }

        const ObjectPushConstants push = makePushConstants(drawable);
        commandList.pushConstants(pipeline_->layout(), VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                                  sizeof(push), &push);
        commandList.bindDescriptorSet(pipeline_->layout(), 2, materialSet(drawable.material).handle());
        commandList.bindVertexBuffer(drawable.mesh->vertexBuffer());
        commandList.bindIndexBuffer(drawable.mesh->indexBuffer());
        commandList.drawIndexed(drawable.mesh->indexCount());
    }

    if (transparentPipeline_ != nullptr && !transparent.empty()) {
        const glm::vec3 cameraPosition = scene.camera().position();
        std::sort(transparent.begin(), transparent.end(), [&cameraPosition](const Drawable* a, const Drawable* b) {
            const glm::vec3 centerA(a->transform[3]);
            const glm::vec3 centerB(b->transform[3]);
            return glm::dot(centerA - cameraPosition, centerA - cameraPosition) >
                   glm::dot(centerB - cameraPosition, centerB - cameraPosition);
        });

        commandList.bindPipeline(*transparentPipeline_);
        commandList.bindDescriptorSet(transparentPipeline_->layout(), 0, frameSets_[slot].handle());
        commandList.bindDescriptorSet(transparentPipeline_->layout(), 1, shadowSets_[slot].handle());
        for (const Drawable* drawable : transparent) {
            const ObjectPushConstants push = makePushConstants(*drawable);
            commandList.pushConstants(transparentPipeline_->layout(),
                                      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push),
                                      &push);
            commandList.bindDescriptorSet(transparentPipeline_->layout(), 2, materialSet(drawable->material).handle());
            commandList.bindVertexBuffer(drawable->mesh->vertexBuffer());
            commandList.bindIndexBuffer(drawable->mesh->indexBuffer());
            commandList.drawIndexed(drawable->mesh->indexCount());
        }
    }

    commandList.endRendering();
}

} // namespace sim::render
