// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/Swapchain.hpp"

#include <algorithm>
#include <stdexcept>

#include "core/Log.hpp"

namespace sim::rhi {

Swapchain::Swapchain(Device& device, const SwapchainDesc& desc)
    : device_(device), surface_(device.surface()), vsync_(desc.vsync) {
    extent_ = chooseExtent(desc.extent);
    create();
}

Swapchain::~Swapchain() {
    destroy();
}

void Swapchain::create() {
    VkSurfaceCapabilitiesKHR capabilities{};
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device_.physicalDevice(), surface_, &capabilities);

    const VkSurfaceFormatKHR surfaceFormat = chooseSurfaceFormat();
    const VkPresentModeKHR presentMode = choosePresentMode(vsync_);

    std::uint32_t imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) {
        imageCount = capabilities.maxImageCount;
    }

    VkSwapchainCreateInfoKHR createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = surface_;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = {extent_.width, extent_.height};
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    const std::uint32_t families[] = {device_.graphicsFamily(), device_.presentFamily()};
    if (device_.graphicsFamily() != device_.presentFamily()) {
        createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        createInfo.queueFamilyIndexCount = 2;
        createInfo.pQueueFamilyIndices = families;
    } else {
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    createInfo.preTransform = capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = presentMode;
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = swapchain_;

    VkSwapchainKHR newSwapchain = VK_NULL_HANDLE;
    if (vkCreateSwapchainKHR(device_.handle(), &createInfo, nullptr, &newSwapchain) != VK_SUCCESS) {
        throw std::runtime_error("failed to create the swapchain");
    }

    destroy();

    swapchain_ = newSwapchain;
    format_ = surfaceFormat.format;
    colorSpace_ = surfaceFormat.colorSpace;

    std::uint32_t actualCount = 0;
    vkGetSwapchainImagesKHR(device_.handle(), swapchain_, &actualCount, nullptr);
    images_.resize(actualCount);
    vkGetSwapchainImagesKHR(device_.handle(), swapchain_, &actualCount, images_.data());

    imageViews_.resize(actualCount);
    for (std::uint32_t i = 0; i < actualCount; ++i) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = images_[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format_;
        viewInfo.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

        if (vkCreateImageView(device_.handle(), &viewInfo, nullptr, &imageViews_[i]) != VK_SUCCESS) {
            throw std::runtime_error("failed to create a swapchain image view");
        }
    }

    core::Log::get("rhi")->info("swapchain {}x{}, {} images, {} present mode", extent_.width, extent_.height,
                                actualCount, presentMode == VK_PRESENT_MODE_FIFO_KHR ? "FIFO" : "low-latency");
}

void Swapchain::destroy() {
    for (VkImageView view : imageViews_) {
        vkDestroyImageView(device_.handle(), view, nullptr);
    }
    imageViews_.clear();
    images_.clear();

    if (swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device_.handle(), swapchain_, nullptr);
        swapchain_ = VK_NULL_HANDLE;
    }
}

void Swapchain::recreate(Extent2D extent, bool vsync) {
    device_.waitIdle();
    extent_ = chooseExtent(extent);
    vsync_ = vsync;
    if (extent_.width == 0 || extent_.height == 0) {
        return; // Minimised: keep the old swapchain until it can be sized again.
    }
    create();
}

VkSurfaceFormatKHR Swapchain::chooseSurfaceFormat() const {
    std::uint32_t count = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device_.physicalDevice(), surface_, &count, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(count);
    vkGetPhysicalDeviceSurfaceFormatsKHR(device_.physicalDevice(), surface_, &count, formats.data());

    for (const VkSurfaceFormatKHR& format : formats) {
        if (format.format == VK_FORMAT_B8G8R8A8_SRGB && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return format;
        }
    }
    return formats.front();
}

VkPresentModeKHR Swapchain::choosePresentMode(bool vsync) const {
    if (vsync) {
        return VK_PRESENT_MODE_FIFO_KHR;
    }

    std::uint32_t count = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device_.physicalDevice(), surface_, &count, nullptr);
    std::vector<VkPresentModeKHR> modes(count);
    vkGetPhysicalDeviceSurfacePresentModesKHR(device_.physicalDevice(), surface_, &count, modes.data());

    for (VkPresentModeKHR mode : modes) {
        if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
            return mode;
        }
    }
    for (VkPresentModeKHR mode : modes) {
        if (mode == VK_PRESENT_MODE_IMMEDIATE_KHR) {
            return mode;
        }
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}

Extent2D Swapchain::chooseExtent(Extent2D requested) const {
    VkSurfaceCapabilitiesKHR capabilities{};
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device_.physicalDevice(), surface_, &capabilities);

    if (capabilities.currentExtent.width != 0xFFFFFFFFu) {
        return {capabilities.currentExtent.width, capabilities.currentExtent.height};
    }
    return {std::clamp(requested.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width),
            std::clamp(requested.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height)};
}

bool Swapchain::acquireNextImage(VkSemaphore signalSemaphore, std::uint32_t& outImageIndex) {
    const VkResult result = vkAcquireNextImageKHR(device_.handle(), swapchain_, UINT64_MAX, signalSemaphore,
                                                  VK_NULL_HANDLE, &outImageIndex);
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        framebufferResized_ = true;
        return false;
    }
    if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("failed to acquire a swapchain image");
    }
    return true;
}

bool Swapchain::present(VkSemaphore waitSemaphore, std::uint32_t imageIndex) {
    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = &waitSemaphore;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain_;
    presentInfo.pImageIndices = &imageIndex;

    const VkResult result = vkQueuePresentKHR(device_.presentQueue(), &presentInfo);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        framebufferResized_ = true;
        return false;
    }
    if (result != VK_SUCCESS) {
        throw std::runtime_error("failed to present the swapchain image");
    }
    return true;
}

} // namespace sim::rhi
