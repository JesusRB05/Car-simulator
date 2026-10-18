// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors

// Depth-only pass that fills the directional shadow map. It reuses the frame
// uniform buffer (set 0) for the light's view-projection and the same push
// constants as the scene pass, so the two pipelines can share the vertex layout.
//
// Reversed-Z, matching the main pass: the near plane maps to 1 and the far
// plane to 0.

struct FrameUniforms
{
    column_major float4x4 viewProjection;
    column_major float4x4 lightViewProjection;
    column_major float4x4 shadowMatrix;
    float4 cameraPosition;
    float4 lightDirection;
    float4 lightColor;
    float4 ambientSky;
    float4 ambientGround;
};

struct PushConstants
{
    column_major float4x4 model;
    float4 baseColor;
    float4 emissive;
    float4 params;
};

ConstantBuffer<FrameUniforms> frame : register(b0);
[[vk::push_constant]] ConstantBuffer<PushConstants> push;

[shader("vertex")]
float4 vertexMain(float3 position : POSITION) : SV_Position
{
    return mul(frame.lightViewProjection, mul(push.model, float4(position, 1.0)));
}

[shader("fragment")]
float4 fragmentMain() : SV_Target
{
    return float4(1.0, 1.0, 1.0, 1.0);
}
