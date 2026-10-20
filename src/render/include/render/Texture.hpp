// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "rhi/Resources.hpp"

namespace sim::asset {
struct Image;
} // namespace sim::asset

namespace sim::rhi {
class Device;
} // namespace sim::rhi

namespace sim::render {

// A 2D texture uploaded once and sampled by the material shaders. The mip chain
// is generated on the CPU with a 2x2 box filter, because neither the content nor
// the loader ships one. Colour textures are uploaded into an sRGB format so the
// hardware linearises on sample, exactly like the swapchain does.
//
// The texture owns its own sampler: the glTF samplers in the base content all
// agree (linear, mipmapped, repeat), so one sampler per texture is enough today
// and keeps the descriptor writes simple.
class Texture {
public:
    // `rgba8` is a tightly packed top-row-first RGBA base level.
    Texture(rhi::Device& device, std::uint32_t width, std::uint32_t height, const std::uint8_t* rgba8, bool srgb,
            bool repeat = true);
    // Convenience overload over a decoded asset image (its `srgb` flag decides
    // the format).
    Texture(rhi::Device& device, const asset::Image& image, bool repeat = true);
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    VkImageView view() const { return image_->view(); }
    VkSampler sampler() const { return sampler_->handle(); }
    std::uint32_t mipLevels() const { return image_->mipLevels(); }

    // Average linear colour of the base level, for flat fallbacks and tools.
    const glm::vec3& averageColor() const { return averageColor_; }

private:
    std::unique_ptr<rhi::Image> image_;
    std::unique_ptr<rhi::Sampler> sampler_;
    glm::vec3 averageColor_{1.0f};
};

// Number of mip levels a width x height image needs to reach 1x1.
std::uint32_t mipLevelCount(std::uint32_t width, std::uint32_t height);

// Builds the full mip chain of an RGBA8 base level into `out` (the levels are
// stored one after another) and fills `levels` with each level's extent.
// `levels[0]` is `base` copied verbatim; every following level is the 2x2 box
// average of the previous one, with the last row/column clamped when the
// dimensions are odd. Exposed so the filter can be tested without a device.
void buildMipChain(const std::uint8_t* base, std::uint32_t width, std::uint32_t height, std::vector<std::uint8_t>& out,
                   std::vector<rhi::Extent2D>& levels);

} // namespace sim::render
