// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors

// Physically-based metallic-roughness shading for the scene pass.
//
// Column-major matrices (mul(M, v), column vectors) matching the GLM layout on
// the CPU side; the uniform struct is uploaded verbatim. See docs/CONVENTIONS.md.
//
// The pipeline is reversed-Z (CompareOp::GreaterEqual, depth cleared to 0), so
// the shadow-map test compares against a depth where closer-to-the-light is
// larger. All lighting is computed in linear space and tone mapped at the end;
// the swapchain is sRGB, so the hardware applies the sRGB encode.

struct FrameUniforms
{
    column_major float4x4 viewProjection;
    column_major float4x4 lightViewProjection;
    column_major float4x4 shadowMatrix;
    float4 cameraPosition;
    float4 lightDirection; // xyz: the direction the light travels
    float4 lightColor;     // rgb radiance
    float4 ambientSky;     // rgb sky ambient, a exposure
    float4 ambientGround;  // rgb ground ambient
};

struct PushConstants
{
    column_major float4x4 model;
    float4 baseColor; // rgb albedo, a opacity
    float4 emissive;  // rgb emissive colour, a strength
    float4 params;    // x metallic, y roughness, z clearcoat, w alpha cutoff (>0 enables alpha test)
};

ConstantBuffer<FrameUniforms> frame : register(b0);
// The shadow map is a separate texture and sampler at set 1 (matching the
// renderer's shadowLayout_).
[[vk::binding(0, 1)]] Texture2D<float> shadowMap;
[[vk::binding(1, 1)]] SamplerState shadowSampler;
// Set 2: the material's colour and emissive maps. A material without them is
// bound to a white / black 1x1, so the sample is always valid.
[[vk::binding(0, 2)]] Sampler2D baseColorMap;
[[vk::binding(1, 2)]] Sampler2D emissiveMap;
[[vk::push_constant]] ConstantBuffer<PushConstants> push;

struct VSOutput
{
    float4 position : SV_Position;
    float3 worldPosition : TEXCOORD1;
    float3 normal : NORMAL0;
    float2 uv : TEXCOORD0;
    float4 color : COLOR0;
};

[shader("vertex")]
VSOutput vertexMain(float3 position : POSITION, float3 normal : NORMAL, float2 uv : TEXCOORD0,
                    float4 tangent : TANGENT, float4 color : COLOR0)
{
    VSOutput output;
    float4 world = mul(push.model, float4(position, 1.0));
    output.position = mul(frame.viewProjection, world);
    output.worldPosition = world.xyz;
    output.normal = mul((float3x3)push.model, normal);
    output.uv = uv;
    output.color = color;
    return output;
}

static const float kPi = 3.14159265359;

float distributionGGX(float nDotH, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float denom = nDotH * nDotH * (a2 - 1.0) + 1.0;
    return a2 / max(kPi * denom * denom, 1.0e-7);
}

float geometrySchlickGGX(float nDot, float roughness)
{
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return nDot / max(nDot * (1.0 - k) + k, 1.0e-7);
}

float3 fresnelSchlick(float cosTheta, float3 f0)
{
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

float3 acesToneMap(float3 x)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
}

// Percentage-closer filtered shadow lookup over a 3x3 kernel. `normal` is the
// (already front-face corrected) shading normal, used to offset the sample
// position so the surface does not shadow itself.
float sampleShadow(float3 worldPosition, float3 normal, float nDotL)
{
    // The orthographic box is ~190 m across and the depth ramp 640 m; sliding the
    // sample along the normal by a couple of texels removes most self-shadowing
    // without the peter-panning a large depth bias would cause.
    float4 clip = mul(frame.shadowMatrix, float4(worldPosition + normal * 0.18, 1.0));
    if (clip.w <= 0.0)
    {
        return 1.0;
    }
    float3 ndc = clip.xyz / clip.w;
    float2 uv = float2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0)
    {
        return 1.0;
    }
    float bias = max(0.00035 * (1.0 - nDotL), 0.00008);
    float texel = 1.0 / 2048.0;
    float lit = 0.0;
    [unroll]
    for (int y = -1; y <= 1; ++y)
    {
        [unroll]
        for (int x = -1; x <= 1; ++x)
        {
            float stored = shadowMap.SampleLevel(shadowSampler, uv + float2(x, y) * texel, 0.0);
            // Reversed-Z: the light-stored depth grows towards the light, so a
            // fragment is lit when nothing in front of it is closer to the sun.
            lit += (stored <= ndc.z + bias) ? 1.0 : 0.0;
        }
    }
    return lit / 9.0;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input, bool isFrontFace : SV_IsFrontFace) : SV_Target
{
    const float4 baseSample = baseColorMap.Sample(input.uv);
    float3 albedo = push.baseColor.rgb * input.color.rgb * baseSample.rgb;
    float alpha = push.baseColor.a * input.color.a * baseSample.a;
    if (push.params.w > 0.0 && alpha < push.params.w)
    {
        discard;
    }

    float3 n = normalize(input.normal);
    // The pipeline draws both faces (so thin and open geometry reads correctly);
    // a back face lights as its mirror image.
    n = isFrontFace ? n : -n;

    const float metallic = saturate(push.params.x);
    const float roughness = clamp(push.params.y, 0.045, 1.0);
    const float clearcoat = saturate(push.params.z);

    const float3 lightDir = normalize(-frame.lightDirection.xyz); // direction towards the light
    const float3 view = normalize(frame.cameraPosition.xyz - input.worldPosition);
    const float nDotL = saturate(dot(n, lightDir));
    const float nDotV = saturate(dot(n, view)) + 1.0e-4;
    const float3 halfVector = normalize(lightDir + view);
    const float nDotH = saturate(dot(n, halfVector));
    const float vDotH = saturate(dot(view, halfVector));

    const float3 f0 = lerp(float3(0.04), albedo, metallic);
    const float3 f90 = float3(1.0);
    const float d = distributionGGX(nDotH, roughness);
    const float g = geometrySchlickGGX(nDotL, roughness) * geometrySchlickGGX(nDotV, roughness);
    const float3 f = fresnelSchlick(vDotH, f0);
    const float3 specular = (d * g) * f / max(4.0 * nDotL * nDotV, 1.0e-4);
    const float3 kd = (f90 - f) * (1.0 - metallic);

    const float shadow = sampleShadow(input.worldPosition, n, nDotL);
    float3 direct = (kd * albedo / kPi + specular) * frame.lightColor.rgb * nDotL * shadow;

    const float up = saturate(n.y * 0.5 + 0.5);
    const float3 ambientColor = lerp(frame.ambientGround.rgb, frame.ambientSky.rgb, up);
    float3 ambient = kd * albedo * ambientColor;
    // A cheap environment specular so metals are not black where the sun is not.
    ambient += f0 * ambientColor * (1.0 - roughness) * 0.5;

    if (clearcoat > 0.0)
    {
        const float ccRoughness = 0.06;
        const float dC = distributionGGX(nDotH, ccRoughness);
        const float gC = geometrySchlickGGX(nDotL, ccRoughness) * geometrySchlickGGX(nDotV, ccRoughness);
        const float3 fC = fresnelSchlick(vDotH, float3(0.04));
        const float3 specC = (dC * gC) * fC / max(4.0 * nDotL * nDotV, 1.0e-4);
        direct += specC * frame.lightColor.rgb * nDotL * shadow * clearcoat;
        ambient += fC * ambientColor * clearcoat * 0.35;
    }

    const float3 emissive = push.emissive.rgb * push.emissive.a * emissiveMap.Sample(input.uv).rgb;
    float3 color = direct + ambient + emissive;
    color *= frame.ambientSky.a; // exposure
    color = acesToneMap(color);
    return float4(color, alpha);
}
