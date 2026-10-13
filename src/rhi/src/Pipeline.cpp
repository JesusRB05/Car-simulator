// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/Pipeline.hpp"

#include <array>
#include <map>
#include <stdexcept>

#include "rhi/Conversions.hpp"

namespace sim::rhi {

DescriptorSetLayout::DescriptorSetLayout(Device& device, const std::vector<DescriptorBinding>& bindings)
    : device_(&device), bindings_(bindings) {
    std::vector<VkDescriptorSetLayoutBinding> vkBindings;
    vkBindings.reserve(bindings.size());
    for (const DescriptorBinding& binding : bindings) {
        VkDescriptorSetLayoutBinding vk{};
        vk.binding = binding.binding;
        vk.descriptorType = toVkDescriptorType(binding.type);
        vk.descriptorCount = binding.count;
        vk.stageFlags = binding.stages;
        vkBindings.push_back(vk);
    }

    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = static_cast<std::uint32_t>(vkBindings.size());
    info.pBindings = vkBindings.data();

    if (vkCreateDescriptorSetLayout(device.handle(), &info, nullptr, &layout_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a descriptor set layout");
    }
}

DescriptorSetLayout::~DescriptorSetLayout() {
    if (layout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_->handle(), layout_, nullptr);
    }
}

DescriptorPool::DescriptorPool(Device& device, std::uint32_t maxSets, const std::vector<DescriptorBinding>& bindings)
    : device_(&device) {
    std::map<DescriptorType, std::uint32_t> counts;
    for (const DescriptorBinding& binding : bindings) {
        counts[binding.type] += binding.count;
    }

    std::vector<VkDescriptorPoolSize> sizes;
    for (const auto& [type, count] : counts) {
        VkDescriptorPoolSize size{};
        size.type = toVkDescriptorType(type);
        size.descriptorCount = count * maxSets;
        sizes.push_back(size);
    }

    VkDescriptorPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    info.maxSets = maxSets;
    info.poolSizeCount = static_cast<std::uint32_t>(sizes.size());
    info.pPoolSizes = sizes.data();

    if (vkCreateDescriptorPool(device.handle(), &info, nullptr, &pool_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a descriptor pool");
    }
}

DescriptorPool::~DescriptorPool() {
    if (pool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_->handle(), pool_, nullptr);
    }
}

VkDescriptorSet DescriptorPool::allocate(const DescriptorSetLayout& layout) {
    VkDescriptorSetLayout layouts[] = {layout.handle()};
    VkDescriptorSetAllocateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    info.descriptorPool = pool_;
    info.descriptorSetCount = 1;
    info.pSetLayouts = layouts;

    VkDescriptorSet set = VK_NULL_HANDLE;
    if (vkAllocateDescriptorSets(device_->handle(), &info, &set) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate a descriptor set");
    }
    return set;
}

void DescriptorPool::reset() {
    vkResetDescriptorPool(device_->handle(), pool_, 0);
}

void DescriptorSet::writeBuffer(std::uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset) const {
    VkDescriptorBufferInfo bufferInfo{};
    bufferInfo.buffer = buffer;
    bufferInfo.offset = offset;
    bufferInfo.range = size;

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = set_;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    write.pBufferInfo = &bufferInfo;

    vkUpdateDescriptorSets(device_->handle(), 1, &write, 0, nullptr);
}

void DescriptorSet::writeImage(std::uint32_t binding, VkSampler sampler, VkImageView view, VkImageLayout layout) const {
    VkDescriptorImageInfo imageInfo{};
    imageInfo.sampler = sampler;
    imageInfo.imageView = view;
    imageInfo.imageLayout = layout;

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = set_;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.pImageInfo = &imageInfo;

    vkUpdateDescriptorSets(device_->handle(), 1, &write, 0, nullptr);
}

namespace {

VkPipelineLayout createPipelineLayout(VkDevice device, const std::vector<const DescriptorSetLayout*>& layouts,
                                      std::uint32_t pushConstantSize, VkShaderStageFlags pushConstantStages) {
    std::vector<VkDescriptorSetLayout> setLayouts;
    setLayouts.reserve(layouts.size());
    for (const DescriptorSetLayout* layout : layouts) {
        setLayouts.push_back(layout != nullptr ? layout->handle() : VK_NULL_HANDLE);
    }

    VkPushConstantRange pushRange{};
    pushRange.stageFlags = pushConstantStages;
    pushRange.offset = 0;
    pushRange.size = pushConstantSize;

    VkPipelineLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    info.setLayoutCount = static_cast<std::uint32_t>(setLayouts.size());
    info.pSetLayouts = setLayouts.empty() ? nullptr : setLayouts.data();
    info.pushConstantRangeCount = pushConstantSize > 0 ? 1 : 0;
    info.pPushConstantRanges = pushConstantSize > 0 ? &pushRange : nullptr;

    VkPipelineLayout layout = VK_NULL_HANDLE;
    if (vkCreatePipelineLayout(device, &info, nullptr, &layout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a pipeline layout");
    }
    return layout;
}

} // namespace

GraphicsPipeline::GraphicsPipeline(Device& device, const GraphicsPipelineDesc& desc) : device_(&device) {
    VkShaderStageFlagBits vertexStageBit = VK_SHADER_STAGE_VERTEX_BIT;

    std::array<VkPipelineShaderStageCreateInfo, 2> stages{};
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = desc.vertexStage.module != nullptr ? desc.vertexStage.module->handle() : VK_NULL_HANDLE;
    stages[0].pName = desc.vertexStage.entryPoint;
    (void)vertexStageBit;

    stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = desc.fragmentStage.module != nullptr ? desc.fragmentStage.module->handle() : VK_NULL_HANDLE;
    stages[1].pName = desc.fragmentStage.entryPoint;

    std::vector<VkVertexInputBindingDescription> bindings;
    for (const VertexBinding& binding : desc.vertexBindings) {
        VkVertexInputBindingDescription vk{};
        vk.binding = binding.binding;
        vk.stride = binding.stride;
        vk.inputRate = binding.perInstance ? VK_VERTEX_INPUT_RATE_INSTANCE : VK_VERTEX_INPUT_RATE_VERTEX;
        bindings.push_back(vk);
    }

    std::vector<VkVertexInputAttributeDescription> attributes;
    for (const VertexAttribute& attribute : desc.vertexAttributes) {
        VkVertexInputAttributeDescription vk{};
        vk.location = attribute.location;
        vk.binding = attribute.binding;
        vk.format = toVkVertexFormat(attribute.format);
        vk.offset = attribute.offset;
        attributes.push_back(vk);
    }

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInput.vertexBindingDescriptionCount = static_cast<std::uint32_t>(bindings.size());
    vertexInput.pVertexBindingDescriptions = bindings.empty() ? nullptr : bindings.data();
    vertexInput.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributes.size());
    vertexInput.pVertexAttributeDescriptions = attributes.empty() ? nullptr : attributes.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = toVkTopology(desc.topology);

    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = toVkCullMode(desc.cullMode);
    // Meshes are wound counter-clockwise seen from outside (Mesh::makeCube).
    // The negative-height viewport (CommandList::setViewport) makes screen-up
    // equal world-up, so a front face still appears counter-clockwise on screen
    // and the usual CCW front face applies. A clockwise front face here would
    // cull the outside of every box and show its interior instead.
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = desc.depthTest ? VK_TRUE : VK_FALSE;
    depthStencil.depthWriteEnable = desc.depthWrite ? VK_TRUE : VK_FALSE;
    depthStencil.depthCompareOp = toVkCompareOp(desc.depthCompare);

    VkPipelineColorBlendAttachmentState blendAttachment{};
    applyBlendMode(desc.blendMode, blendAttachment);

    VkPipelineColorBlendStateCreateInfo colorBlend{};
    colorBlend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlend.attachmentCount = 1;
    colorBlend.pAttachments = &blendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPipelineRenderingCreateInfo renderingInfo{};
    renderingInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    renderingInfo.colorAttachmentCount = static_cast<std::uint32_t>(desc.colorFormats.size());
    std::vector<VkFormat> colorFormats;
    for (Format format : desc.colorFormats) {
        colorFormats.push_back(toVkFormat(format));
    }
    renderingInfo.pColorAttachmentFormats = colorFormats.empty() ? nullptr : colorFormats.data();
    renderingInfo.depthAttachmentFormat = toVkFormat(desc.depthFormat);

    layout_ =
        createPipelineLayout(device.handle(), desc.descriptorLayouts, desc.pushConstantSize, desc.pushConstantStages);

    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.pNext = &renderingInfo;
    pipelineInfo.stageCount = static_cast<std::uint32_t>(stages.size());
    pipelineInfo.pStages = stages.data();
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlend;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = layout_;

    if (vkCreateGraphicsPipelines(device.handle(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline_) !=
        VK_SUCCESS) {
        vkDestroyPipelineLayout(device.handle(), layout_, nullptr);
        layout_ = VK_NULL_HANDLE;
        throw std::runtime_error("failed to create a graphics pipeline");
    }
}

GraphicsPipeline::~GraphicsPipeline() {
    if (pipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_->handle(), pipeline_, nullptr);
    }
    if (layout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_->handle(), layout_, nullptr);
    }
}

ComputePipeline::ComputePipeline(Device& device, const ComputePipelineDesc& desc) : device_(&device) {
    VkPipelineShaderStageCreateInfo stage{};
    stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage.module = desc.computeStage.module != nullptr ? desc.computeStage.module->handle() : VK_NULL_HANDLE;
    stage.pName = desc.computeStage.entryPoint;

    layout_ = createPipelineLayout(device.handle(), desc.descriptorLayouts, desc.pushConstantSize,
                                   VK_SHADER_STAGE_COMPUTE_BIT);

    VkComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.stage = stage;
    pipelineInfo.layout = layout_;

    if (vkCreateComputePipelines(device.handle(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline_) !=
        VK_SUCCESS) {
        vkDestroyPipelineLayout(device.handle(), layout_, nullptr);
        layout_ = VK_NULL_HANDLE;
        throw std::runtime_error("failed to create a compute pipeline");
    }
}

ComputePipeline::~ComputePipeline() {
    if (pipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_->handle(), pipeline_, nullptr);
    }
    if (layout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_->handle(), layout_, nullptr);
    }
}

} // namespace sim::rhi
