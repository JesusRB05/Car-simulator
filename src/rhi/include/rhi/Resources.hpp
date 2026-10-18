// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <vk_mem_alloc.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rhi/Device.hpp"
#include "rhi/Types.hpp"

namespace sim::rhi {

// A GPU buffer allocated through VMA. Host-visible memory is mapped once at
// creation; device-local memory is meant to be filled through a staging path.
class Buffer {
public:
    Buffer(Device& device, std::size_t size, BufferUsage usage, MemoryUsage memory = MemoryUsage::CpuToGpu);
    Buffer(Device& device, const void* data, std::size_t size, BufferUsage usage,
           MemoryUsage memory = MemoryUsage::CpuToGpu);
    ~Buffer();

    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;
    Buffer(Buffer&& other) noexcept;
    Buffer& operator=(Buffer&& other) noexcept;

    VkBuffer handle() const { return buffer_; }
    std::size_t size() const { return size_; }
    MemoryUsage memoryUsage() const { return memory_; }
    void* mappedData() const { return mapped_; }

    // Copies into the mapped region. Only valid for host-visible buffers.
    void upload(const void* data, std::size_t size, std::size_t offset = 0);

private:
    void create(std::size_t size, BufferUsage usage, MemoryUsage memory);
    void release();

    Device* device_ = nullptr;
    VkBuffer buffer_ = VK_NULL_HANDLE;
    VmaAllocation allocation_ = VK_NULL_HANDLE;
    std::size_t size_ = 0;
    MemoryUsage memory_ = MemoryUsage::CpuToGpu;
    void* mapped_ = nullptr;
};

// A GPU image plus its default 2D view.
class Image {
public:
    Image(Device& device, Extent2D extent, Format format, ImageUsage usage, bool depthStencil = false);
    // Sampled images may carry a mip chain (used by the texture upload path);
    // the view exposes every level.
    Image(Device& device, Extent2D extent, Format format, ImageUsage usage, std::uint32_t mipLevels,
          bool depthStencil = false);
    ~Image();

    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;

    VkImage handle() const { return image_; }
    VkImageView view() const { return view_; }
    Extent2D extent() const { return extent_; }
    Format format() const { return format_; }
    std::uint32_t mipLevels() const { return mipLevels_; }

private:
    void create(Extent2D extent, Format format, ImageUsage usage, std::uint32_t mipLevels, bool depthStencil);
    void release();

    Device* device_ = nullptr;
    VkImage image_ = VK_NULL_HANDLE;
    VkImageView view_ = VK_NULL_HANDLE;
    VmaAllocation allocation_ = VK_NULL_HANDLE;
    Extent2D extent_{};
    Format format_ = Format::Undefined;
    std::uint32_t mipLevels_ = 1;
};

// A texture sampler with sensible defaults.
class Sampler {
public:
    Sampler(Device& device, bool linear = true, bool repeat = true);
    ~Sampler();

    Sampler(const Sampler&) = delete;
    Sampler& operator=(const Sampler&) = delete;

    VkSampler handle() const { return sampler_; }

private:
    Device* device_ = nullptr;
    VkSampler sampler_ = VK_NULL_HANDLE;
};

// A SPIR-V shader module.
class ShaderModule {
public:
    ShaderModule(Device& device, const std::vector<std::uint32_t>& spirv);
    ShaderModule(Device& device, const std::byte* data, std::size_t byteSize);
    ~ShaderModule();

    ShaderModule(const ShaderModule&) = delete;
    ShaderModule& operator=(const ShaderModule&) = delete;

    VkShaderModule handle() const { return module_; }

private:
    Device* device_ = nullptr;
    VkShaderModule module_ = VK_NULL_HANDLE;
};

} // namespace sim::rhi
