// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <cstdint>
#include <vector>

#include "rhi/Device.hpp"
#include "rhi/Types.hpp"

namespace sim::rhi {

struct SwapchainDesc {
    Extent2D extent;
    bool vsync = true;
};

// Owns the presentable swapchain and its image views. Out-of-date and
// suboptimal results are reported to the caller so a frame can be retried.
class Swapchain {
public:
    Swapchain(Device& device, const SwapchainDesc& desc);
    ~Swapchain();

    Swapchain(const Swapchain&) = delete;
    Swapchain& operator=(const Swapchain&) = delete;

    // Recreates the swapchain for a new size / vsync setting.
    void recreate(Extent2D extent, bool vsync);

    VkSwapchainKHR handle() const { return swapchain_; }
    VkFormat format() const { return format_; }
    Extent2D extent() const { return extent_; }
    std::uint32_t imageCount() const { return static_cast<std::uint32_t>(images_.size()); }

    VkImage image(std::uint32_t index) const { return images_[index]; }
    VkImageView imageView(std::uint32_t index) const { return imageViews_[index]; }

    // Acquires the next image. Returns false when the swapchain is out of date.
    bool acquireNextImage(VkSemaphore signalSemaphore, std::uint32_t& outImageIndex);

    // Presents the image. Returns false when the swapchain must be recreated.
    bool present(VkSemaphore waitSemaphore, std::uint32_t imageIndex);

    bool framebufferResized() const { return framebufferResized_; }
    void clearResizedFlag() { framebufferResized_ = false; }
    void markResized() { framebufferResized_ = true; }

private:
    void create();
    void destroy();
    VkSurfaceFormatKHR chooseSurfaceFormat() const;
    VkPresentModeKHR choosePresentMode(bool vsync) const;
    Extent2D chooseExtent(Extent2D requested) const;

    Device& device_;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkFormat format_ = VK_FORMAT_UNDEFINED;
    VkColorSpaceKHR colorSpace_ = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    Extent2D extent_{};
    bool vsync_ = true;
    bool framebufferResized_ = false;
    std::vector<VkImage> images_;
    std::vector<VkImageView> imageViews_;
};

} // namespace sim::rhi
