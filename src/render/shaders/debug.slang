// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// Instanced debug lines. One instance is a single line segment (its two
// endpoints and a colour); the vertex shader picks the endpoint from
// SV_VertexID and applies the frame's view-projection, so a whole grid or set
// of axes is a single instanced draw call with no per-vertex buffer.

struct FrameUniforms
{
    float4 viewProj0;
    float4 viewProj1;
    float4 viewProj2;
    float4 viewProj3;
};

ConstantBuffer<FrameUniforms> frame : register(b0);

float4 mulRows(float4 r0, float4 r1, float4 r2, float4 r3, float4 v)
{
    return float4(dot(r0, v), dot(r1, v), dot(r2, v), dot(r3, v));
}

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
    output.position = mulRows(frame.viewProj0, frame.viewProj1, frame.viewProj2, frame.viewProj3, float4(world, 1.0));
    output.tint = color;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target
{
    return input.tint;
}
