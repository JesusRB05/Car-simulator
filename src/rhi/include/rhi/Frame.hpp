// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"
#include "rhi/Swapchain.hpp"
#include "rhi/Types.hpp"

namespace sim::rhi {

// Manages CPU/GPU frame synchronisation for the swapchain: a pool of command
// lists, one fence and two semaphores per frame in flight. It also performs the
// swapchain image layout transitions around rendering.
class Frame {
public:
    Frame(Device& device, Swapchain& swapchain, std::uint32_t framesInFlight = 2);
    ~Frame();

    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;

    // Waits for the slot to be free and acquires the next swapchain image.
    // Returns false when the swapchain is out of date (recreate it and retry).
    bool begin();

    // Ends recording, submits and presents.
    void submitAndPresent();

    CommandList& commandList() { return *slots_[slotIndex_].list; }

    std::uint32_t imageIndex() const { return imageIndex_; }
    std::uint32_t slot() const { return slotIndex_; }
    Format colorFormat() const {
        return swapchain_.format() == VK_FORMAT_B8G8R8A8_SRGB ? Format::B8G8R8A8Srgb : Format::B8G8R8A8Unorm;
    }
    Extent2D extent() const { return swapchain_.extent(); }
    VkImage swapchainImage() const { return swapchain_.image(imageIndex_); }
    VkImageView swapchainImageView() const { return swapchain_.imageView(imageIndex_); }

    // Convenience colour attachment targeting the current swapchain image.
    ColorAttachment swapchainAttachment(LoadOp loadOp = LoadOp::Clear, float r = 0.05f, float g = 0.06f,
                                        float b = 0.08f, float a = 1.0f) const;

private:
    struct Slot {
        std::unique_ptr<CommandList> list;
        VkFence fence = VK_NULL_HANDLE;
        VkSemaphore imageAvailable = VK_NULL_HANDLE;
        VkSemaphore renderFinished = VK_NULL_HANDLE;
    };

    Device& device_;
    Swapchain& swapchain_;
    std::vector<Slot> slots_;
    std::uint32_t slotIndex_ = 0;
    std::uint32_t imageIndex_ = 0;
};

} // namespace sim::rhi
