// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <vk_mem_alloc.h>

#include "rhi/Types.hpp"

namespace sim::rhi {

VkFormat toVkFormat(Format format);
VkFormat toVkVertexFormat(VertexFormat format);
VkBufferUsageFlags toVkBufferUsage(BufferUsage usage);
VmaMemoryUsage toVmaMemoryUsage(MemoryUsage usage);
VkImageUsageFlags toVkImageUsage(ImageUsage usage);
VkImageLayout toVkImageLayout(ImageLayout layout);
VkDescriptorType toVkDescriptorType(DescriptorType type);
VkCompareOp toVkCompareOp(CompareOp op);
VkCullModeFlags toVkCullMode(CullMode mode);
VkPrimitiveTopology toVkTopology(PrimitiveTopology topology);
void applyBlendMode(BlendMode mode, VkPipelineColorBlendAttachmentState& outBlend);

} // namespace sim::rhi
