// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <SDL3/SDL.h>
#include <volk.h>

#include <cstdint>

namespace sim::rhi {
class Device;
class Swapchain;
} // namespace sim::rhi

namespace sim::app {

// Thin owner of the ImGui context and its SDL3 + Vulkan backends. The Vulkan
// backend is built with IMGUI_IMPL_VULKAN_USE_VOLK (see cmake/ImGuiBackends.cmake)
// and dynamic rendering, matching the RHI.
class ImGuiLayer {
public:
    ImGuiLayer(rhi::Device& device, rhi::Swapchain& swapchain, SDL_Window* window);
    ~ImGuiLayer();

    ImGuiLayer(const ImGuiLayer&) = delete;
    ImGuiLayer& operator=(const ImGuiLayer&) = delete;

    // Must be called whenever the swapchain is recreated.
    void onSwapchainRecreated(rhi::Swapchain& swapchain);

    bool processEvent(const SDL_Event& event);

    void beginFrame();
    void endFrame(VkCommandBuffer commandBuffer);

private:
    rhi::Device* device_ = nullptr;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkFormat colorFormat_ = VK_FORMAT_UNDEFINED;
};

} // namespace sim::app
