// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// Instanced debug lines. One instance is a single line segment (its two
// endpoints and a colour); the vertex shader picks the endpoint from
// SV_VertexID and applies the frame's view-projection, so a whole grid or set
// of axes is a single instanced draw call with no per-vertex buffer.
//
// The view-projection is a column-major matrix multiplied as mul(M, v),
// matching the GLM layout uploaded from C++ (see docs/CONVENTIONS.md).

struct FrameUniforms
{
    column_major float4x4 viewProjection;
};

ConstantBuffer<FrameUniforms> frame : register(b0);

struct VSOutput
{
    float4 position : SV_Position;
    [[vk::location(0)]] float4 tint : COLOR0;
};

[shader("vertex")]
VSOutput vertexMain(uint vertexID : SV_VertexID, [[vk::location(0)]] float3 start : START,
                    [[vk::location(1)]] float3 end : END, [[vk::location(2)]] float4 color : COLOR0)
{
    VSOutput output;
    float3 world = vertexID == 0 ? start : end;
    output.position = mul(frame.viewProjection, float4(world, 1.0));
    output.tint = color;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target
{
    return input.tint;
}