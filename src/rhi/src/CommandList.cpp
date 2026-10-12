// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/CommandList.hpp"

#include <stdexcept>

#include "rhi/Conversions.hpp"

namespace sim::rhi {
namespace {

VkPipelineStageFlags2 stageFor(ImageLayout layout)
{
    switch (layout)
    {
    case ImageLayout::ColorAttachment:
        return VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    case ImageLayout::DepthAttachment:
        return VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    case ImageLayout::ShaderReadOnly:
        return VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
    case ImageLayout::TransferSrc:
    case ImageLayout::TransferDst:
        return VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    case ImageLayout::General:
        return VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    default:
        return VK_PIPELINE_STAGE_2_NONE;
    }
}

VkAccessFlags2 accessFor(ImageLayout layout)
{
    switch (layout)
    {
    case ImageLayout::ColorAttachment:
        return VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    case ImageLayout::DepthAttachment:
        return VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    case ImageLayout::ShaderReadOnly:
        return VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
    case ImageLayout::TransferSrc:
        return VK_ACCESS_2_TRANSFER_READ_BIT;
    case ImageLayout::TransferDst:
        return VK_ACCESS_2_TRANSFER_WRITE_BIT;
    case ImageLayout::General:
        return VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    default:
        return VK_ACCESS_2_NONE;
    }
}

} // namespace

CommandList::CommandList(Device& device, bool transient) : device_(&device)
{
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (transient)
    {
        poolInfo.flags |= VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    }
    poolInfo.queueFamilyIndex = device.graphicsFamily();

    if (vkCreateCommandPool(device.handle(), &poolInfo, nullptr, &pool_) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to create a command pool");
    }

    VkCommandBufferAllocateInfo allocateInfo{};
    allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocateInfo.commandPool = pool_;
    allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocateInfo.commandBufferCount = 1;

    if (vkAllocateCommandBuffers(device.handle(), &allocateInfo, &commandBuffer_) != VK_SUCCESS)
    {
        vkDestroyCommandPool(device.handle(), pool_, nullptr);
        pool_ = VK_NULL_HANDLE;
        throw std::runtime_error("failed to allocate a command buffer");
    }
}

CommandList::~CommandList()
{
    if (pool_ != VK_NULL_HANDLE)
    {
        vkDestroyCommandPool(device_->handle(), pool_, nullptr);
    }
}

void CommandList::begin()
{
    vkResetCommandPool(device_->handle(), pool_, 0);

    VkCommandBufferBeginInfo info{};
    info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (vkBeginCommandBuffer(commandBuffer_, &info) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to begin a command buffer");
    }
}

void CommandList::end()
{
    if (vkEndCommandBuffer(commandBuffer_) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to end a command buffer");
    }
}

void CommandList::beginRendering(const std::vector<ColorAttachment>& colorAttachments, const DepthAttachment* depth, Extent2D area)
{
    std::vector<VkRenderingAttachmentInfo> colors;
    colors.reserve(colorAttachments.size());
    for (const ColorAttachment& attachment : colorAttachments)
    {
        VkRenderingAttachmentInfo info{};
        info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
        info.imageView = reinterpret_cast<VkImageView>(attachment.image);
        info.imageLayout = toVkImageLayout(attachment.layout);
        info.loadOp = attachment.loadOp == LoadOp::Load      ? VK_ATTACHMENT_LOAD_OP_LOAD
                      : attachment.loadOp == LoadOp::Clear ? VK_ATTACHMENT_LOAD_OP_CLEAR
                                                           : VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        info.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        info.clearValue.color = {{attachment.clearColor[0], attachment.clearColor[1], attachment.clearColor[2], attachment.clearColor[3]}};
        colors.push_back(info);
    }

    VkRenderingAttachmentInfo depthInfo{};
    if (depth != nullptr)
    {
        depthInfo.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
        depthInfo.imageView = reinterpret_cast<VkImageView>(depth->image);
        depthInfo.imageLayout = toVkImageLayout(depth->layout);
        depthInfo.loadOp = depth->loadOp == LoadOp::Load      ? VK_ATTACHMENT_LOAD_OP_LOAD
                           : depth->loadOp == LoadOp::Clear ? VK_ATTACHMENT_LOAD_OP_CLEAR
                                                            : VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthInfo.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthInfo.clearValue.depthStencil = {depth->clearDepth, depth->clearStencil};
    }

    VkRenderingInfo renderingInfo{};
    renderingInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    renderingInfo.renderArea = {{0, 0}, {area.width, area.height}};
    renderingInfo.layerCount = 1;
    renderingInfo.colorAttachmentCount = static_cast<std::uint32_t>(colors.size());
    renderingInfo.pColorAttachments = colors.empty() ? nullptr : colors.data();
    renderingInfo.pDepthAttachment = depth != nullptr ? &depthInfo : nullptr;

    vkCmdBeginRendering(commandBuffer_, &renderingInfo);
}

void CommandList::endRendering()
{
    vkCmdEndRendering(commandBuffer_);
}

void CommandList::bindPipeline(const GraphicsPipeline& pipeline)
{
    vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.handle());
}

void CommandList::bindComputePipeline(const ComputePipeline& pipeline)
{
    vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline.handle());
}

void CommandList::bindDescriptorSet(VkPipelineLayout layout, std::uint32_t set, VkDescriptorSet descriptorSet,
                                    const std::vector<std::uint32_t>& dynamicOffsets)
{
    vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, set, 1, &descriptorSet,
                            static_cast<std::uint32_t>(dynamicOffsets.size()), dynamicOffsets.empty() ? nullptr : dynamicOffsets.data());
}

void CommandList::pushConstants(VkPipelineLayout layout, VkShaderStageFlags stages, std::uint32_t offset, std::uint32_t size, const void* data)
{
    vkCmdPushConstants(commandBuffer_, layout, stages, offset, size, data);
}

void CommandList::bindVertexBuffer(const Buffer& buffer, std::uint32_t binding)
{
    VkBuffer handle = buffer.handle();
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(commandBuffer_, binding, 1, &handle, &offset);
}

void CommandList::bindIndexBuffer(const Buffer& buffer, bool use32Bit)
{
    vkCmdBindIndexBuffer(commandBuffer_, buffer.handle(), 0, use32Bit ? VK_INDEX_TYPE_UINT32 : VK_INDEX_TYPE_UINT16);
}

void CommandList::setViewport(float x, float y, float width, float height)
{
    VkViewport viewport{};
    viewport.x = x;
    viewport.y = y;
    viewport.width = width;
    viewport.height = height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(commandBuffer_, 0, 1, &viewport);
}

void CommandList::setScissor(std::int32_t x, std::int32_t y, std::uint32_t width, std::uint32_t height)
{
    VkRect2D scissor{};
    scissor.offset = {x, y};
    scissor.extent = {width, height};
    vkCmdSetScissor(commandBuffer_, 0, 1, &scissor);
}

void CommandList::draw(std::uint32_t vertexCount, std::uint32_t firstVertex)
{
    vkCmdDraw(commandBuffer_, vertexCount, 1, firstVertex, 0);
}

void CommandList::drawInstanced(std::uint32_t vertexCount, std::uint32_t instanceCount, std::uint32_t firstVertex,
                                std::uint32_t firstInstance)
{
    vkCmdDraw(commandBuffer_, vertexCount, instanceCount, firstVertex, firstInstance);
}

void CommandList::drawIndexed(std::uint32_t indexCount, std::uint32_t firstIndex, std::int32_t vertexOffset)
{
    vkCmdDrawIndexed(commandBuffer_, indexCount, 1, firstIndex, vertexOffset, 0);
}

void CommandList::dispatch(std::uint32_t groupX, std::uint32_t groupY, std::uint32_t groupZ)
{
    vkCmdDispatch(commandBuffer_, groupX, groupY, groupZ);
}

void CommandList::transitionImage(VkImage image, ImageLayout from, ImageLayout to, VkImageAspectFlags aspect)
{
    VkImageMemoryBarrier2 barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    barrier.srcStageMask = stageFor(from);
    barrier.srcAccessMask = accessFor(from);
    barrier.dstStageMask = stageFor(to);
    barrier.dstAccessMask = accessFor(to);
    barrier.oldLayout = toVkImageLayout(from);
    barrier.newLayout = toVkImageLayout(to);
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange = {aspect, 0, 1, 0, 1};

    VkDependencyInfo dependency{};
    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dependency.imageMemoryBarrierCount = 1;
    dependency.pImageMemoryBarriers = &barrier;

    vkCmdPipelineBarrier2(commandBuffer_, &dependency);
}

void CommandList::copyBuffer(VkBuffer source, VkBuffer destination, VkDeviceSize size)
{
    VkBufferCopy region{};
    region.size = size;
    vkCmdCopyBuffer(commandBuffer_, source, destination, 1, &region);
}

void CommandList::copyImageToBuffer(VkImage image, VkBuffer destination, Extent2D extent)
{
    VkBufferImageCopy region{};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.imageOffset = {0, 0, 0};
    region.imageExtent = {extent.width, extent.height, 1};
    vkCmdCopyImageToBuffer(commandBuffer_, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, destination, 1, &region);
}

} // namespace sim::rhi
