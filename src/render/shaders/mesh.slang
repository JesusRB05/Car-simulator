// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors

struct FrameUniforms
{
    float4 viewProj0;
    float4 viewProj1;
    float4 viewProj2;
    float4 viewProj3;
    float4 cameraPosition;
    float4 lightDirection;
    float4 lightColor;
};

struct PushConstants
{
    float4 model0;
    float4 model1;
    float4 model2;
    float4 model3;
    float4 baseColor;
};

ConstantBuffer<FrameUniforms> frame : register(b0);
[[vk::push_constant]] ConstantBuffer<PushConstants> push;

float4 mulRows(float4 r0, float4 r1, float4 r2, float4 r3, float4 v)
{
    return float4(dot(r0, v), dot(r1, v), dot(r2, v), dot(r3, v));
}

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
    float4 world = mulRows(push.model0, push.model1, push.model2, push.model3, float4(position, 1.0));
    output.position = mulRows(frame.viewProj0, frame.viewProj1, frame.viewProj2, frame.viewProj3, world);
    output.worldNormal = float3(dot(push.model0.xyz, normal), dot(push.model1.xyz, normal), dot(push.model2.xyz, normal));
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
