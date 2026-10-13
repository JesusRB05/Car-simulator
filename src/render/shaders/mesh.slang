// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors

// Column-major matrices (mul(M, v), column vectors) matching the GLM layout on
// the CPU side; the uniform struct is uploaded verbatim. See docs/CONVENTIONS.md.

struct FrameUniforms
{
    column_major float4x4 viewProjection;
    float4 cameraPosition;
    float4 lightDirection;
    float4 lightColor;
};

struct PushConstants
{
    column_major float4x4 model;
    float4 baseColor;
};

ConstantBuffer<FrameUniforms> frame : register(b0);
[[vk::push_constant]] ConstantBuffer<PushConstants> push;

struct VSOutput
{
    float4 position : SV_Position;
    float3 worldNormal : NORMAL0;
    float2 uv : TEXCOORD0;
};

[shader("vertex")]
VSOutput vertexMain(float3 position : POSITION, float3 normal : NORMAL, float2 uv : TEXCOORD0)
{
    VSOutput output;
    float4 world = mul(push.model, float4(position, 1.0));
    output.position = mul(frame.viewProjection, world);
    output.worldNormal = mul(push.model, float4(normal, 0.0)).xyz;
    output.uv = uv;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target
{
    float3 n = normalize(input.worldNormal);
    float3 l = normalize(-frame.lightDirection.xyz);
    float ndl = saturate(dot(n, l));
    // Constant ambient plus a small sky term so up-facing surfaces are never
    // pitch black, then a single Lambert directional term.
    float3 ambient = float3(0.22, 0.23, 0.26) + 0.18 * saturate(n.y);
    float3 color = push.baseColor.rgb * (ambient + ndl * frame.lightColor.rgb);
    return float4(color, push.baseColor.a);
}