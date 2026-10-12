// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// Single translation unit that compiles the Vulkan Memory Allocator
// implementation. volk is included first so that VK_NO_PROTOTYPES is defined
// and VMA switches to dynamically loaded Vulkan functions (the loader symbols
// are supplied by volk through VmaVulkanFunctions in Device::createAllocator).
#include <volk.h>

#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>
