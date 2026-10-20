// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/Device.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <set>
#include <stdexcept>
#include <vector>

#include "core/Log.hpp"
#include "rhi/CommandList.hpp"

namespace sim::rhi {
namespace {

constexpr const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT severity,
                                             VkDebugUtilsMessageTypeFlagsEXT /*types*/,
                                             const VkDebugUtilsMessengerCallbackDataEXT* data, void* userData) {
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        if (auto* device = static_cast<Device*>(userData)) {
            device->countValidationMessage();
        }
    }

    auto logger = core::Log::get("vulkan");
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
        logger->error("{}", data->pMessage);
    } else if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        logger->warn("{}", data->pMessage);
    } else {
        logger->debug("{}", data->pMessage);
    }
    return VK_FALSE;
}

bool hasLayer(const char* name) {
    std::uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    for (const VkLayerProperties& layer : layers) {
        if (std::strcmp(layer.layerName, name) == 0) {
            return true;
        }
    }
    return false;
}

bool deviceSupportsExtensions(VkPhysicalDevice device, bool requireSwapchain) {
    if (!requireSwapchain) {
        return true;
    }

    std::uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data());

    return std::any_of(extensions.begin(), extensions.end(), [](const VkExtensionProperties& ext) {
        return std::strcmp(ext.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0;
    });
}

struct QueueSelection {
    bool complete = false;
    std::uint32_t graphics = 0;
    std::uint32_t compute = 0;
    std::uint32_t transfer = 0;
    std::uint32_t present = 0;
    bool hasGraphics = false;
    bool hasCompute = false;
    bool hasPresent = false;
};

QueueSelection selectQueues(VkPhysicalDevice device, VkSurfaceKHR surface) {
    std::uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());

    QueueSelection selection;
    for (std::uint32_t i = 0; i < count; ++i) {
        const VkQueueFlags flags = families[i].queueFlags;
        if (!selection.hasGraphics && (flags & VK_QUEUE_GRAPHICS_BIT) != 0) {
            selection.graphics = i;
            selection.hasGraphics = true;
        }

        if (surface != VK_NULL_HANDLE) {
            VkBool32 presentSupport = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);
            if (!selection.hasPresent && presentSupport == VK_TRUE) {
                selection.present = i;
                selection.hasPresent = true;
            }
        }
    }

    // Prefer a dedicated compute family, otherwise fall back to graphics.
    selection.compute = selection.graphics;
    for (std::uint32_t i = 0; i < count; ++i) {
        const VkQueueFlags flags = families[i].queueFlags;
        if ((flags & VK_QUEUE_COMPUTE_BIT) != 0 && (flags & VK_QUEUE_GRAPHICS_BIT) == 0) {
            selection.compute = i;
            selection.hasCompute = true;
            break;
        }
    }
    if (!selection.hasCompute) {
        selection.hasCompute = selection.hasGraphics;
    }

    // Prefer a dedicated transfer family, otherwise fall back to graphics.
    selection.transfer = selection.graphics;
    for (std::uint32_t i = 0; i < count; ++i) {
        const VkQueueFlags flags = families[i].queueFlags;
        if ((flags & VK_QUEUE_TRANSFER_BIT) != 0 && (flags & (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT)) == 0) {
            selection.transfer = i;
            break;
        }
    }

    selection.complete = selection.hasGraphics;
    return selection;
}

int scoreDevice(VkPhysicalDevice device, const QueueSelection& queues, VkPhysicalDeviceProperties properties,
                bool requirePresent) {
    if (!queues.complete) {
        return -1;
    }
    if (requirePresent && !queues.hasPresent) {
        return -1;
    }
    if (properties.apiVersion < VK_API_VERSION_1_3) {
        return -1;
    }
    if (!deviceSupportsExtensions(device, requirePresent)) {
        return -1;
    }
    int score = 0;
    if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
        score += 1000;
    } else if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
        score += 500;
    }
    score += static_cast<int>(properties.limits.maxImageDimension2D / 1024);
    return score;
}

} // namespace

Device::Device(const DeviceConfig& config) {
    headless_ = config.headless;
    createInstance(config);
    if (surface_ == VK_NULL_HANDLE && !headless_ && config.createSurface) {
        surface_ = config.createSurface(instance_);
        if (surface_ == VK_NULL_HANDLE) {
            throw std::runtime_error("the platform failed to create a Vulkan surface");
        }
    }
    pickPhysicalDevice();
    createLogicalDevice();
    createAllocator();
    setupDebugMessenger(config);
    core::Log::get("vulkan")->info(
        "using GPU '{}' (Vulkan {}.{}.{})", properties_.deviceName, VK_API_VERSION_MAJOR(properties_.apiVersion),
        VK_API_VERSION_MINOR(properties_.apiVersion), VK_API_VERSION_PATCH(properties_.apiVersion));
}

Device::~Device() {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }
    if (allocator_ != VK_NULL_HANDLE) {
        vmaDestroyAllocator(allocator_);
        allocator_ = VK_NULL_HANDLE;
    }
    if (debugMessenger_ != VK_NULL_HANDLE) {
        vkDestroyDebugUtilsMessengerEXT(instance_, debugMessenger_, nullptr);
        debugMessenger_ = VK_NULL_HANDLE;
    }
    if (device_ != VK_NULL_HANDLE) {
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
    }
}

void Device::createInstance(const DeviceConfig& config) {
    if (volkInitialize() != VK_SUCCESS) {
        throw std::runtime_error("failed to load the Vulkan loader (vulkan-1.dll missing?)");
    }

    validationEnabled_ = config.enableValidation && hasLayer(kValidationLayer);
    if (config.enableValidation && !validationEnabled_) {
        core::Log::get("vulkan")->warn("validation requested but '{}' is not installed; continuing without it",
                                       kValidationLayer);
    }

    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Car Simulator";
    appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    appInfo.pEngineName = "sim";
    appInfo.engineVersion = VK_MAKE_VERSION(0, 1, 0);
    appInfo.apiVersion = VK_API_VERSION_1_3;

    std::vector<const char*> extensions = config.extraInstanceExtensions;
    if (validationEnabled_) {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    createInfo.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

    std::vector<const char*> layers;
    if (validationEnabled_) {
        layers.push_back(kValidationLayer);
    }
    createInfo.enabledLayerCount = static_cast<std::uint32_t>(layers.size());
    createInfo.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();

    if (vkCreateInstance(&createInfo, nullptr, &instance_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create the Vulkan instance");
    }
    volkLoadInstance(instance_);
}

void Device::pickPhysicalDevice() {
    std::uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance_, &count, nullptr);
    if (count == 0) {
        throw std::runtime_error("no Vulkan-capable GPU was found");
    }
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance_, &count, devices.data());

    const bool requirePresent = !headless_ && surface_ != VK_NULL_HANDLE;

    int bestScore = -1;
    VkPhysicalDevice best = VK_NULL_HANDLE;
    QueueSelection bestQueues;

    for (VkPhysicalDevice candidate : devices) {
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(candidate, &properties);
        const QueueSelection queues = selectQueues(candidate, surface_);
        const int score = scoreDevice(candidate, queues, properties, requirePresent);
        if (score > bestScore) {
            bestScore = score;
            best = candidate;
            bestQueues = queues;
        }
    }

    if (best == VK_NULL_HANDLE || bestScore < 0) {
        throw std::runtime_error("no suitable GPU supports Vulkan 1.3 with dynamic rendering");
    }

    physicalDevice_ = best;
    queues_.graphics = bestQueues.graphics;
    queues_.compute = bestQueues.compute;
    queues_.transfer = bestQueues.transfer;
    queues_.present = bestQueues.hasPresent ? bestQueues.present : bestQueues.graphics;
    vkGetPhysicalDeviceProperties(physicalDevice_, &properties_);
    vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memoryProperties_);
}

void Device::createLogicalDevice() {
    VkPhysicalDeviceVulkan12Features supported12{};
    supported12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    VkPhysicalDeviceVulkan13Features supported13{};
    supported13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    supported12.pNext = &supported13;

    VkPhysicalDeviceFeatures2 supportedFeatures{};
    supportedFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    supportedFeatures.pNext = &supported12;
    vkGetPhysicalDeviceFeatures2(physicalDevice_, &supportedFeatures);

    if (supported13.dynamicRendering != VK_TRUE) {
        throw std::runtime_error("the GPU does not support dynamic rendering (Vulkan 1.3 core)");
    }
    if (supported13.synchronization2 != VK_TRUE) {
        throw std::runtime_error("the GPU does not support synchronization2 (Vulkan 1.3 core)");
    }

    VkPhysicalDeviceVulkan13Features enable13{};
    enable13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    enable13.dynamicRendering = VK_TRUE;
    enable13.synchronization2 = VK_TRUE;
    enable13.maintenance4 = supported13.maintenance4;

    VkPhysicalDeviceVulkan12Features enable12{};
    enable12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    enable12.pNext = &enable13;
    enable12.descriptorIndexing = supported12.descriptorIndexing;
    enable12.timelineSemaphore = supported12.timelineSemaphore;
    enable12.scalarBlockLayout = supported12.scalarBlockLayout;
    enable12.bufferDeviceAddress = supported12.bufferDeviceAddress;
    enable12.shaderSampledImageArrayNonUniformIndexing = supported12.shaderSampledImageArrayNonUniformIndexing;

    VkPhysicalDeviceFeatures2 enableFeatures{};
    enableFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    enableFeatures.pNext = &enable12;
    enableFeatures.features.samplerAnisotropy = supportedFeatures.features.samplerAnisotropy;
    enableFeatures.features.fillModeNonSolid = supportedFeatures.features.fillModeNonSolid;

    const std::set<std::uint32_t> uniqueFamilies = {queues_.graphics, queues_.compute, queues_.transfer,
                                                    queues_.present};
    const float priority = 1.0f;
    std::vector<VkDeviceQueueCreateInfo> queueInfos;
    for (std::uint32_t family : uniqueFamilies) {
        VkDeviceQueueCreateInfo queueInfo{};
        queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueInfo.queueFamilyIndex = family;
        queueInfo.queueCount = 1;
        queueInfo.pQueuePriorities = &priority;
        queueInfos.push_back(queueInfo);
    }

    std::vector<const char*> deviceExtensions;
    if (!headless_) {
        deviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    }

    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.pNext = &enableFeatures;
    createInfo.queueCreateInfoCount = static_cast<std::uint32_t>(queueInfos.size());
    createInfo.pQueueCreateInfos = queueInfos.data();
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.empty() ? nullptr : deviceExtensions.data();

    if (vkCreateDevice(physicalDevice_, &createInfo, nullptr, &device_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create the Vulkan device");
    }
    volkLoadDevice(device_);

    vkGetDeviceQueue(device_, queues_.graphics, 0, &graphicsQueue_);
    vkGetDeviceQueue(device_, queues_.compute, 0, &computeQueue_);
    vkGetDeviceQueue(device_, queues_.transfer, 0, &transferQueue_);
    vkGetDeviceQueue(device_, queues_.present, 0, &presentQueue_);
}

void Device::createAllocator() {
    VmaVulkanFunctions vulkanFunctions{};
    vulkanFunctions.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
    vulkanFunctions.vkGetDeviceProcAddr = vkGetDeviceProcAddr;

    VmaAllocatorCreateInfo allocatorInfo{};
    allocatorInfo.physicalDevice = physicalDevice_;
    allocatorInfo.device = device_;
    allocatorInfo.instance = instance_;
    allocatorInfo.vulkanApiVersion = VK_API_VERSION_1_3;
    allocatorInfo.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
    allocatorInfo.pVulkanFunctions = &vulkanFunctions;

    if (vmaCreateAllocator(&allocatorInfo, &allocator_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create the VMA allocator");
    }
}

void Device::setupDebugMessenger(const DeviceConfig& config) {
    if (!validationEnabled_ || !config.enableValidation) {
        return;
    }
    VkDebugUtilsMessengerCreateInfoEXT createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    createInfo.messageSeverity =
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                             VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                             VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    createInfo.pfnUserCallback = &debugCallback;
    createInfo.pUserData = this;

    if (vkCreateDebugUtilsMessengerEXT(instance_, &createInfo, nullptr, &debugMessenger_) != VK_SUCCESS) {
        core::Log::get("vulkan")->warn("failed to install the debug messenger");
    }
}

void Device::waitIdle() const {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }
}

void Device::submitImmediate(CommandList& commandList) {
    VkFence fence = VK_NULL_HANDLE;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (vkCreateFence(device_, &fenceInfo, nullptr, &fence) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a one-shot fence");
    }

    VkCommandBufferSubmitInfo commandInfo{};
    commandInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
    commandInfo.commandBuffer = commandList.handle();

    VkSubmitInfo2 submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
    submitInfo.commandBufferInfoCount = 1;
    submitInfo.pCommandBufferInfos = &commandInfo;

    const VkResult submit = vkQueueSubmit2(graphicsQueue_, 1, &submitInfo, fence);
    if (submit == VK_SUCCESS) {
        vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
    }
    vkDestroyFence(device_, fence, nullptr);
    if (submit != VK_SUCCESS) {
        throw std::runtime_error("failed to submit a one-shot command list");
    }
}

} // namespace sim::rhi
