// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/DebugDraw.hpp"

#include <cstddef>

#include "core/Log.hpp"
#include "render/Uniforms.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::render {
namespace {

const rhi::DescriptorBinding kUniformBinding{0, rhi::DescriptorType::UniformBuffer, 1,
                                             VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT};

} // namespace

DebugDraw::DebugDraw(rhi::Device& device, vfs::VirtualFileSystem& vfs, std::uint32_t maxInstances)
    : device_(device), vfs_(vfs), shaders_(device, vfs), maxInstances_(maxInstances)
{
    layout_ = std::make_unique<rhi::DescriptorSetLayout>(device_, std::vector<rhi::DescriptorBinding>{kUniformBinding});
    pool_ = std::make_unique<rhi::DescriptorPool>(device_, 1, layout_->bindings());

    uniformBuffer_ = std::make_unique<rhi::Buffer>(device_, sizeof(glm::mat4), rhi::BufferUsage::Uniform);
    instanceBuffer_ = std::make_unique<rhi::Buffer>(device_, sizeof(Instance) * maxInstances_, rhi::BufferUsage::Vertex);

    rhi::DescriptorSet set(&device_, pool_->allocate(*layout_));
    set.writeBuffer(0, uniformBuffer_->handle(), sizeof(glm::mat4));
    set_ = set;

    instances_.reserve(maxInstances_);
}

DebugDraw::~DebugDraw() = default;

void DebugDraw::ensureTargets(rhi::Format colorFormat, rhi::Extent2D extent)
{
    extent_ = extent;
    if (pipeline_ != nullptr && colorFormat == colorFormat_)
    {
        return;
    }
    colorFormat_ = colorFormat;
    createPipeline(colorFormat, extent);
}

void DebugDraw::createPipeline(rhi::Format colorFormat, rhi::Extent2D)
{
    const rhi::ShaderModule* vertex = shaders_.get("shaders/debug.vertex.spv");
    const rhi::ShaderModule* fragment = shaders_.get("shaders/debug.fragment.spv");
    if (vertex == nullptr || fragment == nullptr)
    {
        core::Log::get("render")->error("debug draw: missing shaders/debug.{vertex,fragment}.spv");
        pipeline_.reset();
        return;
    }

    rhi::GraphicsPipelineDesc desc;
    desc.vertexBindings = {{0, sizeof(Instance), true}};
    desc.vertexAttributes = {{0, 0, rhi::VertexFormat::Float3, offsetof(Instance, start)},
                             {1, 0, rhi::VertexFormat::Float3, offsetof(Instance, end)},
                             {2, 0, rhi::VertexFormat::Float4, offsetof(Instance, color)}};
    desc.vertexStage = {vertex, "vertexMain"};
    desc.fragmentStage = {fragment, "fragmentMain"};
    desc.topology = rhi::PrimitiveTopology::LineList;
    desc.cullMode = rhi::CullMode::None;
    desc.depthTest = false;
    desc.depthWrite = false;
    desc.blendMode = rhi::BlendMode::Opaque;
    desc.colorFormats = {colorFormat};
    desc.descriptorLayouts = {layout_.get()};

    pipeline_ = std::make_unique<rhi::GraphicsPipeline>(device_, desc);
}

void DebugDraw::addLine(const glm::vec3& start, const glm::vec3& end, const glm::vec4& color)
{
    if (instances_.size() >= maxInstances_)
    {
        return;
    }
    instances_.push_back({start, end, color});
}

void DebugDraw::addAxes(const glm::vec3& origin, float length)
{
    addLine(origin, origin + glm::vec3(length, 0.0f, 0.0f), {1.0f, 0.25f, 0.25f, 1.0f});
    addLine(origin, origin + glm::vec3(0.0f, length, 0.0f), {0.25f, 1.0f, 0.25f, 1.0f});
    addLine(origin, origin + glm::vec3(0.0f, 0.0f, length), {0.3f, 0.5f, 1.0f, 1.0f});
}

void DebugDraw::addGrid(float halfSize, float step, const glm::vec4& lineColor, const glm::vec4& axisColor)
{
    const int steps = static_cast<int>(halfSize / step);
    const float extent = static_cast<float>(steps) * step;
    for (int i = -steps; i <= steps; ++i)
    {
        const float offset = static_cast<float>(i) * step;
        const glm::vec4 color = i == 0 ? axisColor : lineColor;
        addLine({-extent, 0.0f, offset}, {extent, 0.0f, offset}, color);
        addLine({offset, 0.0f, -extent}, {offset, 0.0f, extent}, color);
    }
}

void DebugDraw::render(rhi::CommandList& commandList, const Camera& camera, const rhi::ColorAttachment& colorAttachment)
{
    if (instances_.empty())
    {
        return;
    }
    if (pipeline_ == nullptr)
    {
        createPipeline(colorFormat_, extent_);
        if (pipeline_ == nullptr)
        {
            instances_.clear();
            return;
        }
    }

    const std::uint32_t count = static_cast<std::uint32_t>(instances_.size());
    const glm::mat4 viewProj = camera.viewProjection();
    struct
    {
        alignas(16) glm::vec4 rows[4];
    } frame{};
    for (int row = 0; row < 4; ++row)
    {
        frame.rows[row] = matrixRow(viewProj, row);
    }
    uniformBuffer_->upload(&frame, sizeof(frame));
    instanceBuffer_->upload(instances_.data(), sizeof(Instance) * count);

    commandList.beginRendering({colorAttachment}, nullptr, extent_);
    commandList.bindPipeline(*pipeline_);
    commandList.bindDescriptorSet(pipeline_->layout(), 0, set_.handle());
    commandList.setViewport(0.0f, 0.0f, static_cast<float>(extent_.width), static_cast<float>(extent_.height));
    commandList.setScissor(0, 0, extent_.width, extent_.height);
    commandList.bindVertexBuffer(*instanceBuffer_);
    commandList.drawInstanced(2, count);
    commandList.endRendering();

    instances_.clear();
}

} // namespace sim::render
