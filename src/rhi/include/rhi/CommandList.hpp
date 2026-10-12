// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <cstdint>
#include <vector>

#include "rhi/Device.hpp"
#include "rhi/Pipeline.hpp"
#include "rhi/Resources.hpp"
#include "rhi/Types.hpp"

namespace sim::rhi {

// A single primary command buffer together with its own pool. The typical use
// is: begin() -> record -> end() -> submit through Frame or the queue helpers.
class CommandList
{
public:
    explicit CommandList(Device& device, bool transient = true);
    ~CommandList();

    CommandList(const CommandList&) = delete;
    CommandList& operator=(const CommandList&) = delete;

    VkCommandBuffer handle() const { return commandBuffer_; }

    void begin();
    void end();

    void beginRendering(const std::vector<ColorAttachment>& colorAttachments, const DepthAttachment* depth, Extent2D area);
    void endRendering();

    void bindPipeline(const GraphicsPipeline& pipeline);
    void bindComputePipeline(const ComputePipeline& pipeline);
    void bindDescriptorSet(VkPipelineLayout layout, std::uint32_t set, VkDescriptorSet descriptorSet,
                           const std::vector<std::uint32_t>& dynamicOffsets = {});
    void pushConstants(VkPipelineLayout layout, VkShaderStageFlags stages, std::uint32_t offset, std::uint32_t size, const void* data);

    void bindVertexBuffer(const Buffer& buffer, std::uint32_t binding = 0);
    void bindIndexBuffer(const Buffer& buffer, bool use32Bit = true);

    void setViewport(float x, float y, float width, float height);
    void setScissor(std::int32_t x, std::int32_t y, std::uint32_t width, std::uint32_t height);

    void draw(std::uint32_t vertexCount, std::uint32_t firstVertex = 0);
    void drawInstanced(std::uint32_t vertexCount, std::uint32_t instanceCount, std::uint32_t firstVertex = 0,
                       std::uint32_t firstInstance = 0);
    void drawIndexed(std::uint32_t indexCount, std::uint32_t firstIndex = 0, std::int32_t vertexOffset = 0);
    void dispatch(std::uint32_t groupX, std::uint32_t groupY, std::uint32_t groupZ);

    // Synchronisation2 image layout transition (the caller picks the aspect).
    void transitionImage(VkImage image, ImageLayout from, ImageLayout to, VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT);
    void copyBuffer(VkBuffer source, VkBuffer destination, VkDeviceSize size);
    // Copies a colour image into host-visible memory. The image must already be
    // in the TransferSrc layout (transition it with transitionImage first).
    void copyImageToBuffer(VkImage image, VkBuffer destination, Extent2D extent);

private:
    Device* device_ = nullptr;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer_ = VK_NULL_HANDLE;
};

} // namespace sim::rhi
