// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/Conversions.hpp"

#include "rhi/Types.hpp"

namespace sim::rhi {

std::uint32_t vertexFormatSize(VertexFormat format) {
    switch (format) {
    case VertexFormat::Float:
    case VertexFormat::Int:
    case VertexFormat::UInt:
        return 4;
    case VertexFormat::Float2:
    case VertexFormat::Int2:
    case VertexFormat::UInt2:
        return 8;
    case VertexFormat::Float3:
    case VertexFormat::Int3:
    case VertexFormat::UInt3:
        return 12;
    case VertexFormat::Float4:
    case VertexFormat::Int4:
    case VertexFormat::UInt4:
        return 16;
    }
    return 0;
}

VkFormat toVkFormat(Format format) {
    switch (format) {
    case Format::R8G8B8A8Unorm:
        return VK_FORMAT_R8G8B8A8_UNORM;
    case Format::R8G8B8A8Srgb:
        return VK_FORMAT_R8G8B8A8_SRGB;
    case Format::B8G8R8A8Unorm:
        return VK_FORMAT_B8G8R8A8_UNORM;
    case Format::B8G8R8A8Srgb:
        return VK_FORMAT_B8G8R8A8_SRGB;
    case Format::R16G16B16A16Sfloat:
        return VK_FORMAT_R16G16B16A16_SFLOAT;
    case Format::R32G32B32Sfloat:
        return VK_FORMAT_R32G32B32_SFLOAT;
    case Format::R32G32Sfloat:
        return VK_FORMAT_R32G32_SFLOAT;
    case Format::R32Uint:
        return VK_FORMAT_R32_UINT;
    case Format::D32Sfloat:
        return VK_FORMAT_D32_SFLOAT;
    case Format::D24UnormS8Uint:
        return VK_FORMAT_D24_UNORM_S8_UINT;
    case Format::D32SfloatS8Uint:
        return VK_FORMAT_D32_SFLOAT_S8_UINT;
    case Format::Undefined:
        break;
    }
    return VK_FORMAT_UNDEFINED;
}

VkFormat toVkVertexFormat(VertexFormat format) {
    switch (format) {
    case VertexFormat::Float:
        return VK_FORMAT_R32_SFLOAT;
    case VertexFormat::Float2:
        return VK_FORMAT_R32G32_SFLOAT;
    case VertexFormat::Float3:
        return VK_FORMAT_R32G32B32_SFLOAT;
    case VertexFormat::Float4:
        return VK_FORMAT_R32G32B32A32_SFLOAT;
    case VertexFormat::Int:
        return VK_FORMAT_R32_SINT;
    case VertexFormat::Int2:
        return VK_FORMAT_R32G32_SINT;
    case VertexFormat::Int3:
        return VK_FORMAT_R32G32B32_SINT;
    case VertexFormat::Int4:
        return VK_FORMAT_R32G32B32A32_SINT;
    case VertexFormat::UInt:
        return VK_FORMAT_R32_UINT;
    case VertexFormat::UInt2:
        return VK_FORMAT_R32G32_UINT;
    case VertexFormat::UInt3:
        return VK_FORMAT_R32G32B32_UINT;
    case VertexFormat::UInt4:
        return VK_FORMAT_R32G32B32A32_UINT;
    }
    return VK_FORMAT_UNDEFINED;
}

VkBufferUsageFlags toVkBufferUsage(BufferUsage usage) {
    VkBufferUsageFlags flags = 0;
    if (hasFlag(usage, BufferUsage::Vertex)) {
        flags |= VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    }
    if (hasFlag(usage, BufferUsage::Index)) {
        flags |= VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    }
    if (hasFlag(usage, BufferUsage::Uniform)) {
        flags |= VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    }
    if (hasFlag(usage, BufferUsage::Storage)) {
        flags |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    }
    if (hasFlag(usage, BufferUsage::Indirect)) {
        flags |= VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
    }
    if (hasFlag(usage, BufferUsage::TransferSrc)) {
        flags |= VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    }
    if (hasFlag(usage, BufferUsage::TransferDst)) {
        flags |= VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    }
    return flags;
}

VmaMemoryUsage toVmaMemoryUsage(MemoryUsage usage) {
    switch (usage) {
    case MemoryUsage::GpuOnly:
        return VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    case MemoryUsage::CpuToGpu:
        return VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
    case MemoryUsage::GpuToCpu:
        return VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
    }
    return VMA_MEMORY_USAGE_AUTO;
}

VkImageUsageFlags toVkImageUsage(ImageUsage usage) {
    VkImageUsageFlags flags = 0;
    if (hasFlag(usage, ImageUsage::Sampled)) {
        flags |= VK_IMAGE_USAGE_SAMPLED_BIT;
    }
    if (hasFlag(usage, ImageUsage::Storage)) {
        flags |= VK_IMAGE_USAGE_STORAGE_BIT;
    }
    if (hasFlag(usage, ImageUsage::ColorAttachment)) {
        flags |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    }
    if (hasFlag(usage, ImageUsage::DepthStencilAttachment)) {
        flags |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    }
    if (hasFlag(usage, ImageUsage::TransferSrc)) {
        flags |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    }
    if (hasFlag(usage, ImageUsage::TransferDst)) {
        flags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    }
    return flags;
}

VkImageLayout toVkImageLayout(ImageLayout layout) {
    switch (layout) {
    case ImageLayout::ColorAttachment:
        return VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    case ImageLayout::DepthAttachment:
        return VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    case ImageLayout::ShaderReadOnly:
        return VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    case ImageLayout::TransferSrc:
        return VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    case ImageLayout::TransferDst:
        return VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    case ImageLayout::Present:
        return VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    case ImageLayout::General:
        return VK_IMAGE_LAYOUT_GENERAL;
    case ImageLayout::Undefined:
        break;
    }
    return VK_IMAGE_LAYOUT_UNDEFINED;
}

VkDescriptorType toVkDescriptorType(DescriptorType type) {
    switch (type) {
    case DescriptorType::UniformBuffer:
        return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    case DescriptorType::StorageBuffer:
        return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    case DescriptorType::CombinedImageSampler:
        return VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    }
    return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
}

VkCompareOp toVkCompareOp(CompareOp op) {
    switch (op) {
    case CompareOp::Never:
        return VK_COMPARE_OP_NEVER;
    case CompareOp::Less:
        return VK_COMPARE_OP_LESS;
    case CompareOp::Equal:
        return VK_COMPARE_OP_EQUAL;
    case CompareOp::LessEqual:
        return VK_COMPARE_OP_LESS_OR_EQUAL;
    case CompareOp::Greater:
        return VK_COMPARE_OP_GREATER;
    case CompareOp::NotEqual:
        return VK_COMPARE_OP_NOT_EQUAL;
    case CompareOp::GreaterEqual:
        return VK_COMPARE_OP_GREATER_OR_EQUAL;
    case CompareOp::Always:
        return VK_COMPARE_OP_ALWAYS;
    }
    return VK_COMPARE_OP_LESS_OR_EQUAL;
}

VkCullModeFlags toVkCullMode(CullMode mode) {
    switch (mode) {
    case CullMode::None:
        return VK_CULL_MODE_NONE;
    case CullMode::Front:
        return VK_CULL_MODE_FRONT_BIT;
    case CullMode::Back:
        return VK_CULL_MODE_BACK_BIT;
    }
    return VK_CULL_MODE_BACK_BIT;
}

VkPrimitiveTopology toVkTopology(PrimitiveTopology topology) {
    switch (topology) {
    case PrimitiveTopology::TriangleList:
        return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    case PrimitiveTopology::TriangleStrip:
        return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
    case PrimitiveTopology::LineList:
        return VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    }
    return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
}

void applyBlendMode(BlendMode mode, VkPipelineColorBlendAttachmentState& outBlend) {
    outBlend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    switch (mode) {
    case BlendMode::Opaque:
        outBlend.blendEnable = VK_FALSE;
        break;
    case BlendMode::AlphaBlend:
        outBlend.blendEnable = VK_TRUE;
        outBlend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        outBlend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        outBlend.colorBlendOp = VK_BLEND_OP_ADD;
        outBlend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        outBlend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        outBlend.alphaBlendOp = VK_BLEND_OP_ADD;
        break;
    case BlendMode::Additive:
        outBlend.blendEnable = VK_TRUE;
        outBlend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        outBlend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
        outBlend.colorBlendOp = VK_BLEND_OP_ADD;
        outBlend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        outBlend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        outBlend.alphaBlendOp = VK_BLEND_OP_ADD;
        break;
    }
}

std::string formatName(Format format) {
    switch (format) {
    case Format::R8G8B8A8Unorm:
        return "R8G8B8A8_UNORM";
    case Format::R8G8B8A8Srgb:
        return "R8G8B8A8_SRGB";
    case Format::B8G8R8A8Unorm:
        return "B8G8R8A8_UNORM";
    case Format::B8G8R8A8Srgb:
        return "B8G8R8A8_SRGB";
    case Format::R16G16B16A16Sfloat:
        return "R16G16B16A16_SFLOAT";
    case Format::D32Sfloat:
        return "D32_SFLOAT";
    case Format::D24UnormS8Uint:
        return "D24_UNORM_S8_UINT";
    default:
        return "UNDEFINED";
    }
}

} // namespace sim::rhi
