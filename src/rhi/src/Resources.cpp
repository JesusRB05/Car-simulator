// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/Resources.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

#include "rhi/Conversions.hpp"

namespace sim::rhi {

Buffer::Buffer(Device& device, std::size_t size, BufferUsage usage, MemoryUsage memory) : device_(&device) {
    create(size, usage, memory);
}

Buffer::Buffer(Device& device, const void* data, std::size_t size, BufferUsage usage, MemoryUsage memory)
    : device_(&device) {
    create(size, usage, memory);
    if (data != nullptr && mapped_ != nullptr) {
        std::memcpy(mapped_, data, size);
    }
}

Buffer::Buffer(Buffer&& other) noexcept {
    *this = std::move(other);
}

Buffer& Buffer::operator=(Buffer&& other) noexcept {
    if (this != &other) {
        release();
        device_ = other.device_;
        buffer_ = other.buffer_;
        allocation_ = other.allocation_;
        size_ = other.size_;
        memory_ = other.memory_;
        mapped_ = other.mapped_;
        other.buffer_ = VK_NULL_HANDLE;
        other.allocation_ = VK_NULL_HANDLE;
        other.mapped_ = nullptr;
        other.size_ = 0;
    }
    return *this;
}

void Buffer::create(std::size_t size, BufferUsage usage, MemoryUsage memory) {
    size_ = size;
    memory_ = memory;

    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = toVkBufferUsage(usage);
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocationInfo{};
    allocationInfo.usage = toVmaMemoryUsage(memory);
    if (memory == MemoryUsage::CpuToGpu) {
        allocationInfo.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    } else if (memory == MemoryUsage::GpuToCpu) {
        allocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    }

    VmaAllocationInfo resultInfo{};
    if (vmaCreateBuffer(device_->allocator(), &bufferInfo, &allocationInfo, &buffer_, &allocation_, &resultInfo) !=
        VK_SUCCESS) {
        throw std::runtime_error("failed to create a buffer");
    }
    mapped_ = resultInfo.pMappedData;
}

void Buffer::upload(const void* data, std::size_t size, std::size_t offset) {
    if (mapped_ == nullptr) {
        throw std::runtime_error("upload() called on a buffer that is not host-visible");
    }
    if (offset + size > size_) {
        throw std::runtime_error("buffer upload out of range");
    }
    std::memcpy(static_cast<std::byte*>(mapped_) + offset, data, size);
    vmaFlushAllocation(device_->allocator(), allocation_, offset, size);
}

void Buffer::release() {
    if (buffer_ != VK_NULL_HANDLE) {
        vmaDestroyBuffer(device_->allocator(), buffer_, allocation_);
        buffer_ = VK_NULL_HANDLE;
        allocation_ = VK_NULL_HANDLE;
        mapped_ = nullptr;
    }
}

Buffer::~Buffer() {
    release();
}

Image::Image(Device& device, Extent2D extent, Format format, ImageUsage usage, bool depthStencil)
    : device_(&device), extent_(extent), format_(format) {
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent = {extent.width, extent.height, 1};
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = toVkFormat(format);
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = toVkImageUsage(usage);
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocationInfo{};
    allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

    if (vmaCreateImage(device.allocator(), &imageInfo, &allocationInfo, &image_, &allocation_, nullptr) != VK_SUCCESS) {
        throw std::runtime_error("failed to create an image");
    }

    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = image_;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = imageInfo.format;
    viewInfo.subresourceRange.aspectMask = depthStencil ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(device.handle(), &viewInfo, nullptr, &view_) != VK_SUCCESS) {
        vmaDestroyImage(device.allocator(), image_, allocation_);
        image_ = VK_NULL_HANDLE;
        throw std::runtime_error("failed to create an image view");
    }
}

Image::~Image() {
    release();
}

void Image::release() {
    if (view_ != VK_NULL_HANDLE) {
        vkDestroyImageView(device_->handle(), view_, nullptr);
        view_ = VK_NULL_HANDLE;
    }
    if (image_ != VK_NULL_HANDLE) {
        vmaDestroyImage(device_->allocator(), image_, allocation_);
        image_ = VK_NULL_HANDLE;
        allocation_ = VK_NULL_HANDLE;
    }
}

Sampler::Sampler(Device& device, bool linear, bool repeat) : device_(&device) {
    VkSamplerCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    info.magFilter = linear ? VK_FILTER_LINEAR : VK_FILTER_NEAREST;
    info.minFilter = linear ? VK_FILTER_LINEAR : VK_FILTER_NEAREST;
    info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    info.addressModeU = repeat ? VK_SAMPLER_ADDRESS_MODE_REPEAT : VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    info.addressModeV = info.addressModeU;
    info.addressModeW = info.addressModeU;
    info.maxLod = VK_LOD_CLAMP_NONE;

    if (vkCreateSampler(device.handle(), &info, nullptr, &sampler_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a sampler");
    }
}

Sampler::~Sampler() {
    if (sampler_ != VK_NULL_HANDLE) {
        vkDestroySampler(device_->handle(), sampler_, nullptr);
    }
}

ShaderModule::ShaderModule(Device& device, const std::vector<std::uint32_t>& spirv)
    : ShaderModule(device, reinterpret_cast<const std::byte*>(spirv.data()), spirv.size() * sizeof(std::uint32_t)) {}

ShaderModule::ShaderModule(Device& device, const std::byte* data, std::size_t byteSize) : device_(&device) {
    if (data == nullptr || byteSize == 0 || (byteSize % 4) != 0) {
        throw std::runtime_error("invalid SPIR-V module size");
    }

    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = byteSize;
    createInfo.pCode = reinterpret_cast<const std::uint32_t*>(data);

    if (vkCreateShaderModule(device.handle(), &createInfo, nullptr, &module_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a shader module");
    }
}

ShaderModule::~ShaderModule() {
    if (module_ != VK_NULL_HANDLE) {
        vkDestroyShaderModule(device_->handle(), module_, nullptr);
    }
}

} // namespace sim::rhi
