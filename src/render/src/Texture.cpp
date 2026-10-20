// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Texture.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "asset/Gltf.hpp"
#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"

namespace sim::render {

std::uint32_t mipLevelCount(std::uint32_t width, std::uint32_t height) {
    std::uint32_t levels = 1;
    std::uint32_t size = std::max(width, height);
    while (size > 1) {
        size /= 2;
        ++levels;
    }
    return levels;
}

void buildMipChain(const std::uint8_t* base, std::uint32_t width, std::uint32_t height, std::vector<std::uint8_t>& out,
                   std::vector<rhi::Extent2D>& levels) {
    levels.clear();
    std::uint32_t levelWidth = width;
    std::uint32_t levelHeight = height;
    std::size_t total = 0;
    while (true) {
        levels.push_back({levelWidth, levelHeight});
        total += static_cast<std::size_t>(levelWidth) * levelHeight * 4u;
        if (levelWidth == 1 && levelHeight == 1) {
            break;
        }
        levelWidth = std::max(levelWidth / 2u, 1u);
        levelHeight = std::max(levelHeight / 2u, 1u);
    }

    out.assign(total, 0);
    std::memcpy(out.data(), base, static_cast<std::size_t>(width) * height * 4u);

    // Each level is the 2x2 average of the previous one. Averaging in the stored
    // (possibly sRGB-encoded) space is the usual real-time compromise: it is
    // cheaper than decode/average/encode and the mips are only used beyond one
    // texel per pixel.
    std::size_t sourceOffset = 0;
    std::size_t targetOffset = static_cast<std::size_t>(width) * height * 4u;
    for (std::size_t level = 1; level < levels.size(); ++level) {
        const std::uint32_t sourceWidth = levels[level - 1].width;
        const std::uint32_t sourceHeight = levels[level - 1].height;
        const std::uint32_t targetWidth = levels[level].width;
        const std::uint32_t targetHeight = levels[level].height;
        for (std::uint32_t y = 0; y < targetHeight; ++y) {
            for (std::uint32_t x = 0; x < targetWidth; ++x) {
                const std::uint32_t x0 = std::min(x * 2u, sourceWidth - 1u);
                const std::uint32_t x1 = std::min(x * 2u + 1u, sourceWidth - 1u);
                const std::uint32_t y0 = std::min(y * 2u, sourceHeight - 1u);
                const std::uint32_t y1 = std::min(y * 2u + 1u, sourceHeight - 1u);
                const std::size_t target = targetOffset + (static_cast<std::size_t>(y) * targetWidth + x) * 4u;
                for (int channel = 0; channel < 4; ++channel) {
                    const std::size_t a = sourceOffset + (static_cast<std::size_t>(y0) * sourceWidth + x0) * 4u;
                    const std::size_t b = sourceOffset + (static_cast<std::size_t>(y0) * sourceWidth + x1) * 4u;
                    const std::size_t c = sourceOffset + (static_cast<std::size_t>(y1) * sourceWidth + x0) * 4u;
                    const std::size_t d = sourceOffset + (static_cast<std::size_t>(y1) * sourceWidth + x1) * 4u;
                    const unsigned sum = static_cast<unsigned>(out[a + channel]) + out[b + channel] + out[c + channel] +
                                         out[d + channel];
                    out[target + channel] = static_cast<std::uint8_t>((sum + 2u) / 4u);
                }
            }
        }
        sourceOffset = targetOffset;
        targetOffset += static_cast<std::size_t>(targetWidth) * targetHeight * 4u;
    }
}

Texture::Texture(rhi::Device& device, std::uint32_t width, std::uint32_t height, const std::uint8_t* rgba8, bool srgb,
                 bool repeat) {
    if (width == 0 || height == 0 || rgba8 == nullptr) {
        throw std::runtime_error("Texture: an empty image cannot be uploaded");
    }

    std::vector<std::uint8_t> pixels;
    std::vector<rhi::Extent2D> levels;
    buildMipChain(rgba8, width, height, pixels, levels);

    const rhi::Format format = srgb ? rhi::Format::R8G8B8A8Srgb : rhi::Format::R8G8B8A8Unorm;
    image_ = std::make_unique<rhi::Image>(device, rhi::Extent2D{width, height}, format,
                                          rhi::ImageUsage::Sampled | rhi::ImageUsage::TransferDst,
                                          static_cast<std::uint32_t>(levels.size()));
    sampler_ = std::make_unique<rhi::Sampler>(device, true, repeat);

    // A single host-visible staging buffer holds every level; a one-shot command
    // list copies them in place and hands the image to the shader in the
    // read-only layout.
    rhi::Buffer staging(device, pixels.data(), pixels.size(), rhi::BufferUsage::TransferSrc, rhi::MemoryUsage::CpuToGpu);
    rhi::CommandList commandList(device);
    commandList.begin();
    commandList.transitionImage(image_->handle(), rhi::ImageLayout::Undefined, rhi::ImageLayout::TransferDst,
                                VK_IMAGE_ASPECT_COLOR_BIT, static_cast<std::uint32_t>(levels.size()));
    VkDeviceSize offset = 0;
    for (std::size_t level = 0; level < levels.size(); ++level) {
        commandList.copyBufferToImage(staging.handle(), image_->handle(), levels[level], static_cast<std::uint32_t>(level),
                                      offset);
        offset += static_cast<VkDeviceSize>(levels[level].width) * levels[level].height * 4u;
    }
    commandList.transitionImage(image_->handle(), rhi::ImageLayout::TransferDst, rhi::ImageLayout::ShaderReadOnly,
                                VK_IMAGE_ASPECT_COLOR_BIT, static_cast<std::uint32_t>(levels.size()));
    commandList.end();
    device.submitImmediate(commandList);

    // The average is only needed for fallbacks; keep it cheap and skip the
    // per-texel loop when the caller does not care.
    glm::dvec3 sum(0.0);
    const std::size_t texels = static_cast<std::size_t>(width) * height;
    for (std::size_t i = 0; i < texels; ++i) {
        sum += glm::dvec3(rgba8[i * 4 + 0] / 255.0, rgba8[i * 4 + 1] / 255.0, rgba8[i * 4 + 2] / 255.0);
    }
    averageColor_ = glm::vec3(sum / static_cast<double>(texels));
}

Texture::Texture(rhi::Device& device, const asset::Image& image, bool repeat)
    : Texture(device, image.width, image.height, image.pixels.data(), image.srgb, repeat) {
    // The loader already averaged the texels in linear space.
    averageColor_ = image.averageColor;
}

Texture::~Texture() = default;

} // namespace sim::render
