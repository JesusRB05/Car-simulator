// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sim::rhi {

enum class Format : std::uint32_t
{
    Undefined = 0,
    R8G8B8A8Unorm,
    R8G8B8A8Srgb,
    B8G8R8A8Unorm,
    B8G8R8A8Srgb,
    R16G16B16A16Sfloat,
    R32G32B32Sfloat,
    R32G32Sfloat,
    R32Uint,
    D32Sfloat,
    D24UnormS8Uint,
    D32SfloatS8Uint,
};

enum class BufferUsage : std::uint32_t
{
    None = 0,
    Vertex = 1u << 0,
    Index = 1u << 1,
    Uniform = 1u << 2,
    Storage = 1u << 3,
    Indirect = 1u << 4,
    TransferSrc = 1u << 5,
    TransferDst = 1u << 6,
};

constexpr BufferUsage operator|(BufferUsage a, BufferUsage b)
{
    return static_cast<BufferUsage>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}
constexpr bool hasFlag(BufferUsage value, BufferUsage flag)
{
    return (static_cast<std::uint32_t>(value) & static_cast<std::uint32_t>(flag)) != 0;
}

enum class MemoryUsage
{
    GpuOnly,
    CpuToGpu,
    GpuToCpu,
};

enum class ImageUsage : std::uint32_t
{
    None = 0,
    Sampled = 1u << 0,
    Storage = 1u << 1,
    ColorAttachment = 1u << 2,
    DepthStencilAttachment = 1u << 3,
    TransferSrc = 1u << 4,
    TransferDst = 1u << 5,
};

constexpr ImageUsage operator|(ImageUsage a, ImageUsage b)
{
    return static_cast<ImageUsage>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}
constexpr bool hasFlag(ImageUsage value, ImageUsage flag)
{
    return (static_cast<std::uint32_t>(value) & static_cast<std::uint32_t>(flag)) != 0;
}

enum class ImageLayout
{
    Undefined,
    ColorAttachment,
    DepthAttachment,
    ShaderReadOnly,
    TransferSrc,
    TransferDst,
    Present,
    General,
};

enum class VertexFormat
{
    Float,
    Float2,
    Float3,
    Float4,
    Int,
    Int2,
    Int3,
    Int4,
    UInt,
    UInt2,
    UInt3,
    UInt4,
};

std::uint32_t vertexFormatSize(VertexFormat format);

struct VertexAttribute
{
    std::uint32_t location = 0;
    std::uint32_t binding = 0;
    VertexFormat format = VertexFormat::Float3;
    std::uint32_t offset = 0;
};

struct VertexBinding
{
    std::uint32_t binding = 0;
    std::uint32_t stride = 0;
    bool perInstance = false;
};

struct Extent2D
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

enum class PrimitiveTopology
{
    TriangleList,
    TriangleStrip,
    LineList,
};

enum class CullMode
{
    None,
    Front,
    Back,
};

enum class CompareOp
{
    Never,
    Less,
    Equal,
    LessEqual,
    Greater,
    NotEqual,
    GreaterEqual,
    Always,
};

enum class BlendMode
{
    Opaque,
    AlphaBlend,
    Additive,
};

enum class LoadOp
{
    Load,
    Clear,
    DontCare,
};

struct ColorAttachment
{
    std::uint64_t image = 0; // VkImageView handle
    ImageLayout layout = ImageLayout::ColorAttachment;
    LoadOp loadOp = LoadOp::Clear;
    float clearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
};

struct DepthAttachment
{
    std::uint64_t image = 0; // VkImageView handle
    ImageLayout layout = ImageLayout::DepthAttachment;
    LoadOp loadOp = LoadOp::Clear;
    float clearDepth = 1.0f;
    std::uint32_t clearStencil = 0;
};

enum class DescriptorType
{
    UniformBuffer,
    StorageBuffer,
    CombinedImageSampler,
};

std::string formatName(Format format);

} // namespace sim::rhi
