// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <vk_mem_alloc.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "rhi/Types.hpp"

namespace sim::rhi {

class CommandList;

struct DeviceConfig {
    bool enableValidation = false;
    // Headless mode renders entirely offscreen: no surface is created, no
    // present support is required and the swapchain extension is not enabled.
    // This is what lets software CI (SwiftShader / lavapipe) exercise rendering.
    bool headless = false;
    std::vector<const char*> extraInstanceExtensions;
    // The application supplies the platform code that turns the instance into a
    // presentable surface (SDL in the shipped app). Keeping it here is what lets
    // the RHI stay completely platform-agnostic.
    std::function<VkSurfaceKHR(VkInstance)> createSurface;
};

struct QueueIndices {
    std::uint32_t graphics = 0;
    std::uint32_t compute = 0;
    std::uint32_t transfer = 0;
    std::uint32_t present = 0;
};

// Owns the Vulkan instance, the chosen physical device, the logical device,
// every queue used by the engine and the VMA allocator. This is the single
// point where the Vulkan instance/device lifetime is managed.
class Device {
public:
    Device(const DeviceConfig& config);
    ~Device();

    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    VkInstance instance() const { return instance_; }
    VkPhysicalDevice physicalDevice() const { return physicalDevice_; }
    VkDevice handle() const { return device_; }
    VmaAllocator allocator() const { return allocator_; }
    VkSurfaceKHR surface() const { return surface_; }

    VkQueue graphicsQueue() const { return graphicsQueue_; }
    VkQueue computeQueue() const { return computeQueue_; }
    VkQueue transferQueue() const { return transferQueue_; }
    VkQueue presentQueue() const { return presentQueue_; }

    std::uint32_t graphicsFamily() const { return queues_.graphics; }
    std::uint32_t computeFamily() const { return queues_.compute; }
    std::uint32_t transferFamily() const { return queues_.transfer; }
    std::uint32_t presentFamily() const { return queues_.present; }
    const QueueIndices& queueIndices() const { return queues_; }

    const VkPhysicalDeviceProperties& properties() const { return properties_; }
    const VkPhysicalDeviceMemoryProperties& memoryProperties() const { return memoryProperties_; }

    bool validationEnabled() const { return validationEnabled_; }
    bool headless() const { return headless_; }
    std::string deviceName() const { return properties_.deviceName; }

    // Number of warning/error diagnostics emitted by the validation layers.
    // Headless mode uses this to fail (non-zero exit) on any message.
    void countValidationMessage() { validationMessages_.fetch_add(1, std::memory_order_relaxed); }
    std::uint32_t validationMessageCount() const { return validationMessages_.load(std::memory_order_relaxed); }

    void waitIdle() const;

    // Submits an already-recorded (ended) command list on the graphics queue and
    // waits for it. This is the one-shot path for load-time work that does not
    // fit the per-frame `Frame`: uploading a texture, copying a buffer.
    void submitImmediate(CommandList& commandList);

private:
    void createInstance(const DeviceConfig& config);
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createAllocator();
    void setupDebugMessenger(const DeviceConfig& config);

    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkInstance instance_ = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger_ = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VmaAllocator allocator_ = VK_NULL_HANDLE;

    VkQueue graphicsQueue_ = VK_NULL_HANDLE;
    VkQueue computeQueue_ = VK_NULL_HANDLE;
    VkQueue transferQueue_ = VK_NULL_HANDLE;
    VkQueue presentQueue_ = VK_NULL_HANDLE;

    QueueIndices queues_{};
    VkPhysicalDeviceProperties properties_{};
    VkPhysicalDeviceMemoryProperties memoryProperties_{};
    bool validationEnabled_ = false;
    bool headless_ = false;
    std::atomic<std::uint32_t> validationMessages_{0};
};

} // namespace sim::rhi
