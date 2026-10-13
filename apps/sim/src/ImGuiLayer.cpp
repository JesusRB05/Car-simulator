// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "ImGuiLayer.hpp"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <implot.h>

#include <stdexcept>

#include "rhi/Device.hpp"
#include "rhi/Swapchain.hpp"

namespace sim::app {

ImGuiLayer::ImGuiLayer(rhi::Device& device, rhi::Swapchain& swapchain, SDL_Window* window) : device_(&device) {
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = 1000;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 1000;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;

    if (vkCreateDescriptorPool(device.handle(), &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create the ImGui descriptor pool");
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();

    if (!ImGui_ImplSDL3_InitForVulkan(window)) {
        throw std::runtime_error("failed to initialise the ImGui SDL3 backend");
    }

    colorFormat_ = swapchain.format();

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.ApiVersion = VK_API_VERSION_1_3;
    initInfo.Instance = device.instance();
    initInfo.PhysicalDevice = device.physicalDevice();
    initInfo.Device = device.handle();
    initInfo.QueueFamily = device.graphicsFamily();
    initInfo.Queue = device.graphicsQueue();
    initInfo.DescriptorPool = descriptorPool_;
    initInfo.MinImageCount = swapchain.imageCount() < 2 ? 2 : swapchain.imageCount();
    initInfo.ImageCount = swapchain.imageCount();
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    initInfo.UseDynamicRendering = true;
    initInfo.PipelineInfoMain.PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    initInfo.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
    initInfo.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats = &colorFormat_;

    if (!ImGui_ImplVulkan_Init(&initInfo)) {
        throw std::runtime_error("failed to initialise the ImGui Vulkan backend");
    }
}

ImGuiLayer::~ImGuiLayer() {
    device_->waitIdle();
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    if (descriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_->handle(), descriptorPool_, nullptr);
    }
}

void ImGuiLayer::onSwapchainRecreated(rhi::Swapchain& swapchain) {
    ImGui_ImplVulkan_SetMinImageCount(swapchain.imageCount() < 2 ? 2 : swapchain.imageCount());
}

bool ImGuiLayer::processEvent(const SDL_Event& event) {
    return ImGui_ImplSDL3_ProcessEvent(&event);
}

void ImGuiLayer::beginFrame() {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::endFrame(VkCommandBuffer commandBuffer) {
    ImGui::Render();
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
}

} // namespace sim::app
