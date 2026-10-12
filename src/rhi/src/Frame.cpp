// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/Frame.hpp"

#include <stdexcept>

namespace sim::rhi {

Frame::Frame(Device& device, Swapchain& swapchain, std::uint32_t framesInFlight)
    : device_(device), swapchain_(swapchain)
{
    framesInFlight = framesInFlight < 1 ? 1 : framesInFlight;
    slots_.resize(framesInFlight);

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    for (Slot& slot : slots_)
    {
        slot.list = std::make_unique<CommandList>(device);
        if (vkCreateFence(device.handle(), &fenceInfo, nullptr, &slot.fence) != VK_SUCCESS ||
            vkCreateSemaphore(device.handle(), &semaphoreInfo, nullptr, &slot.imageAvailable) != VK_SUCCESS ||
            vkCreateSemaphore(device.handle(), &semaphoreInfo, nullptr, &slot.renderFinished) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to create frame synchronisation objects");
        }
    }
}

Frame::~Frame()
{
    device_.waitIdle();
    for (Slot& slot : slots_)
    {
        if (slot.imageAvailable != VK_NULL_HANDLE)
        {
            vkDestroySemaphore(device_.handle(), slot.imageAvailable, nullptr);
        }
        if (slot.renderFinished != VK_NULL_HANDLE)
        {
            vkDestroySemaphore(device_.handle(), slot.renderFinished, nullptr);
        }
        if (slot.fence != VK_NULL_HANDLE)
        {
            vkDestroyFence(device_.handle(), slot.fence, nullptr);
        }
    }
}

bool Frame::begin()
{
    Slot& slot = slots_[slotIndex_];
    vkWaitForFences(device_.handle(), 1, &slot.fence, VK_TRUE, UINT64_MAX);

    if (swapchain_.framebufferResized())
    {
        return false;
    }

    std::uint32_t index = 0;
    if (!swapchain_.acquireNextImage(slot.imageAvailable, index))
    {
        return false;
    }
    imageIndex_ = index;

    vkResetFences(device_.handle(), 1, &slot.fence);

    slot.list->begin();
    slot.list->transitionImage(swapchain_.image(imageIndex_), ImageLayout::Undefined, ImageLayout::ColorAttachment);
    return true;
}

void Frame::submitAndPresent()
{
    Slot& slot = slots_[slotIndex_];
    slot.list->transitionImage(swapchain_.image(imageIndex_), ImageLayout::ColorAttachment, ImageLayout::Present);
    slot.list->end();

    VkSemaphoreSubmitInfo waitInfo{};
    waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
    waitInfo.semaphore = slot.imageAvailable;
    waitInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

    VkSemaphoreSubmitInfo signalInfo{};
    signalInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
    signalInfo.semaphore = slot.renderFinished;
    signalInfo.stageMask = VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;

    VkCommandBufferSubmitInfo commandInfo{};
    commandInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
    commandInfo.commandBuffer = slot.list->handle();

    VkSubmitInfo2 submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
    submitInfo.waitSemaphoreInfoCount = 1;
    submitInfo.pWaitSemaphoreInfos = &waitInfo;
    submitInfo.signalSemaphoreInfoCount = 1;
    submitInfo.pSignalSemaphoreInfos = &signalInfo;
    submitInfo.commandBufferInfoCount = 1;
    submitInfo.pCommandBufferInfos = &commandInfo;

    if (vkQueueSubmit2(device_.graphicsQueue(), 1, &submitInfo, slot.fence) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to submit the frame");
    }

    swapchain_.present(slot.renderFinished, imageIndex_);
    slotIndex_ = (slotIndex_ + 1) % slots_.size();
}

ColorAttachment Frame::swapchainAttachment(LoadOp loadOp, float r, float g, float b, float a) const
{
    ColorAttachment attachment;
    attachment.image = reinterpret_cast<std::uint64_t>(swapchain_.imageView(imageIndex_));
    attachment.layout = ImageLayout::ColorAttachment;
    attachment.loadOp = loadOp;
    attachment.clearColor[0] = r;
    attachment.clearColor[1] = g;
    attachment.clearColor[2] = b;
    attachment.clearColor[3] = a;
    return attachment;
}

} // namespace sim::rhi
