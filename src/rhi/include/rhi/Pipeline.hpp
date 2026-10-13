// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <cstdint>
#include <vector>

#include "rhi/Device.hpp"
#include "rhi/Resources.hpp"
#include "rhi/Types.hpp"

namespace sim::rhi {

struct DescriptorBinding {
    std::uint32_t binding = 0;
    DescriptorType type = DescriptorType::UniformBuffer;
    std::uint32_t count = 1;
    VkShaderStageFlags stages = VK_SHADER_STAGE_ALL;
};

class DescriptorSetLayout {
public:
    DescriptorSetLayout(Device& device, const std::vector<DescriptorBinding>& bindings);
    ~DescriptorSetLayout();

    DescriptorSetLayout(const DescriptorSetLayout&) = delete;
    DescriptorSetLayout& operator=(const DescriptorSetLayout&) = delete;

    VkDescriptorSetLayout handle() const { return layout_; }
    const std::vector<DescriptorBinding>& bindings() const { return bindings_; }

private:
    Device* device_ = nullptr;
    VkDescriptorSetLayout layout_ = VK_NULL_HANDLE;
    std::vector<DescriptorBinding> bindings_;
};

class DescriptorPool {
public:
    DescriptorPool(Device& device, std::uint32_t maxSets, const std::vector<DescriptorBinding>& bindings);
    ~DescriptorPool();

    DescriptorPool(const DescriptorPool&) = delete;
    DescriptorPool& operator=(const DescriptorPool&) = delete;

    VkDescriptorSet allocate(const DescriptorSetLayout& layout);
    void reset();

private:
    Device* device_ = nullptr;
    VkDescriptorPool pool_ = VK_NULL_HANDLE;
};

// Thin, updatable wrapper around a VkDescriptorSet.
class DescriptorSet {
public:
    DescriptorSet() = default;
    DescriptorSet(Device* device, VkDescriptorSet set) : device_(device), set_(set) {}
    explicit DescriptorSet(VkDescriptorSet set) : set_(set) {}

    VkDescriptorSet handle() const { return set_; }

    void writeBuffer(std::uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset = 0) const;
    void writeImage(std::uint32_t binding, VkSampler sampler, VkImageView view,
                    VkImageLayout layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) const;

private:
    Device* device_ = nullptr;
    VkDescriptorSet set_ = VK_NULL_HANDLE;
};

struct ShaderStageDesc {
    const ShaderModule* module = nullptr;
    const char* entryPoint = "main";
};

struct GraphicsPipelineDesc {
    std::vector<VertexBinding> vertexBindings;
    std::vector<VertexAttribute> vertexAttributes;
    ShaderStageDesc vertexStage{};
    ShaderStageDesc fragmentStage{};
    PrimitiveTopology topology = PrimitiveTopology::TriangleList;
    CullMode cullMode = CullMode::Back;
    bool depthTest = true;
    bool depthWrite = true;
    CompareOp depthCompare = CompareOp::Less;
    BlendMode blendMode = BlendMode::Opaque;
    std::vector<Format> colorFormats;
    Format depthFormat = Format::Undefined;
    std::vector<const DescriptorSetLayout*> descriptorLayouts;
    std::uint32_t pushConstantSize = 0;
    VkShaderStageFlags pushConstantStages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
};

class GraphicsPipeline {
public:
    GraphicsPipeline(Device& device, const GraphicsPipelineDesc& desc);
    ~GraphicsPipeline();

    GraphicsPipeline(const GraphicsPipeline&) = delete;
    GraphicsPipeline& operator=(const GraphicsPipeline&) = delete;

    VkPipeline handle() const { return pipeline_; }
    VkPipelineLayout layout() const { return layout_; }

private:
    Device* device_ = nullptr;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
};

struct ComputePipelineDesc {
    ShaderStageDesc computeStage{};
    std::vector<const DescriptorSetLayout*> descriptorLayouts;
    std::uint32_t pushConstantSize = 0;
};

class ComputePipeline {
public:
    ComputePipeline(Device& device, const ComputePipelineDesc& desc);
    ~ComputePipeline();

    ComputePipeline(const ComputePipeline&) = delete;
    ComputePipeline& operator=(const ComputePipeline&) = delete;

    VkPipeline handle() const { return pipeline_; }
    VkPipelineLayout layout() const { return layout_; }

private:
    Device* device_ = nullptr;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
};

} // namespace sim::rhi
