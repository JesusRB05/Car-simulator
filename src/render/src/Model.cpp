// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Model.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <utility>

#include "core/Log.hpp"
#include "render/Texture.hpp"
#include "rhi/Device.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::render {
namespace {

// Slot names a skin may use. The long form is the canonical one; the short forms
// are accepted so hand-written descriptors stay readable.
bool isBaseColorSlot(std::string_view slot) {
    return slot == "base_color" || slot == "basecolor" || slot == "albedo" || slot == "body";
}

bool isEmissiveSlot(std::string_view slot) {
    return slot == "emissive" || slot == "emission";
}

} // namespace

Material toRenderMaterial(const asset::Material& source) {
    Material material;
    material.baseColor = source.baseColorFactor;
    material.emissive = source.emissiveFactor;
    material.emissiveStrength = source.emissiveStrength;
    material.metallic = source.metallic;
    material.roughness = source.roughness;
    material.clearcoat = source.clearcoatFactor;
    material.clearcoatRoughness = source.clearcoatRoughness;
    material.alphaCutoff = source.alphaCutoff;
    material.doubleSided = source.doubleSided;
    material.alphaBlend = source.alphaMode == asset::Material::AlphaMode::Blend;
    material.alphaMask = source.alphaMode == asset::Material::AlphaMode::Mask;
    return material;
}

glm::mat4 Model::nodeWorld(int nodeIndex) const {
    if (nodeIndex < 0 || static_cast<std::size_t>(nodeIndex) >= source.nodes.size()) {
        return glm::mat4(1.0f);
    }
    return source.nodes[static_cast<std::size_t>(nodeIndex)].world;
}

std::vector<Material> uploadMaterials(rhi::Device& device, const asset::GltfAsset& asset) {
    // One texture per referenced image, created on demand, with an sRGB view when
    // the image carries colour.
    std::vector<std::shared_ptr<Texture>> imageTextures(asset.images.size());
    const auto textureForImage = [&](int imageIndex) -> std::shared_ptr<Texture> {
        if (imageIndex < 0 || static_cast<std::size_t>(imageIndex) >= asset.images.size()) {
            return nullptr;
        }
        std::shared_ptr<Texture>& texture = imageTextures[static_cast<std::size_t>(imageIndex)];
        const asset::Image& image = asset.images[static_cast<std::size_t>(imageIndex)];
        if (texture == nullptr && !image.pixels.empty()) {
            texture = std::make_shared<Texture>(device, image, /*repeat=*/true);
        }
        return texture;
    };

    std::vector<Material> materials(asset.materials.size());
    for (std::size_t i = 0; i < asset.materials.size(); ++i) {
        const asset::Material& source = asset.materials[i];
        Material& material = materials[i];
        material = toRenderMaterial(source);
        if (source.baseColorTexture >= 0 && static_cast<std::size_t>(source.baseColorTexture) < asset.textures.size()) {
            material.baseColorTexture =
                textureForImage(asset.textures[static_cast<std::size_t>(source.baseColorTexture)].image);
        }
        if (source.emissiveTexture >= 0 && static_cast<std::size_t>(source.emissiveTexture) < asset.textures.size()) {
            material.emissiveTexture =
                textureForImage(asset.textures[static_cast<std::size_t>(source.emissiveTexture)].image);
        }
    }
    return materials;
}

std::shared_ptr<Model> loadModel(rhi::Device& device, const vfs::VirtualFileSystem& vfs, std::string_view virtualPath,
                                 const std::vector<MaterialOverride>& overrides, std::vector<std::string>* warnings) {
    auto log = core::Log::get("render");
    const auto report = [&](const std::string& message) {
        if (warnings != nullptr) {
            warnings->push_back(message);
        }
    };

    asset::GltfLoadResult loaded = asset::loadGltf(vfs, virtualPath);
    for (const std::string& warning : loaded.warnings) {
        report(warning);
    }
    if (!loaded.ok()) {
        for (const std::string& error : loaded.errors) {
            log->error("model: {}", error);
        }
        return nullptr;
    }

    auto model = std::make_shared<Model>();
    model->source = std::move(*loaded.asset);
    asset::GltfAsset& asset = model->source;

    // ---- Materials --------------------------------------------------------
    std::vector<Material> materials = uploadMaterials(device, asset);

    // The override images are decoded through the same path the loader uses for
    // external URIs, so a skin is just another PNG on the VFS.
    std::vector<std::shared_ptr<Texture>> overrideTextures(overrides.size());
    for (std::size_t i = 0; i < overrides.size(); ++i) {
        const MaterialOverride& override = overrides[i];
        if (override.file.empty()) {
            continue;
        }
        std::optional<asset::Image> image = asset::loadImage(vfs, override.file, /*srgb=*/true);
        if (!image) {
            report(std::string(virtualPath) + ": cannot read the skin texture '" + override.file + "'");
            continue;
        }
        overrideTextures[i] = std::make_shared<Texture>(device, *image, /*repeat=*/true);
    }

    for (std::size_t i = 0; i < overrides.size(); ++i) {
        const MaterialOverride& override = overrides[i];
        if (overrideTextures[i] == nullptr) {
            continue;
        }
        bool applied = false;
        for (std::size_t m = 0; m < materials.size(); ++m) {
            if (!override.material.empty() && asset.materials[m].name != override.material) {
                continue;
            }
            if (isBaseColorSlot(override.slot)) {
                materials[m].baseColorTexture = overrideTextures[i];
                applied = true;
            } else if (isEmissiveSlot(override.slot)) {
                materials[m].emissiveTexture = overrideTextures[i];
                applied = true;
            }
        }
        if (!applied) {
            report(std::string(virtualPath) + ": no material matches the skin slot '" + override.slot + "' on '" +
                   override.material + "'");
        }
    }

    // ---- Geometry ---------------------------------------------------------
    for (std::size_t nodeIndex = 0; nodeIndex < asset.nodes.size(); ++nodeIndex) {
        const asset::Node& node = asset.nodes[nodeIndex];
        if (node.mesh < 0 || static_cast<std::size_t>(node.mesh) >= asset.meshes.size()) {
            continue;
        }
        asset::Mesh& mesh = asset.meshes[static_cast<std::size_t>(node.mesh)];
        for (asset::Primitive& primitive : mesh.primitives) {
            if (primitive.vertices.empty() || primitive.indices.empty()) {
                continue;
            }
            ModelPart part;
            // The uploaded vertex layout is the asset's, byte for byte.
            static_assert(sizeof(asset::Vertex) == sizeof(Vertex),
                          "asset::Vertex and render::Vertex must stay layout-compatible");
            std::vector<Vertex> vertices(primitive.vertices.size());
            std::memcpy(vertices.data(), primitive.vertices.data(), vertices.size() * sizeof(Vertex));
            part.mesh = std::make_shared<Mesh>(device, vertices, primitive.indices);
            part.materialIndex = primitive.material;
            if (primitive.material >= 0 && static_cast<std::size_t>(primitive.material) < materials.size()) {
                part.material = materials[static_cast<std::size_t>(primitive.material)];
            }
            part.node = static_cast<int>(nodeIndex);
            model->parts.push_back(std::move(part));

            std::vector<asset::Vertex>().swap(primitive.vertices);
            std::vector<std::uint32_t>().swap(primitive.indices);
        }
    }

    // The pixels are on the GPU now; keep only what the caller may still read
    // (the average colour, the size and the sRGB flag).
    for (asset::Image& image : asset.images) {
        std::vector<std::uint8_t>().swap(image.pixels);
    }

    log->info("model '{}': {} part(s), {} material(s), {} texture(s)", std::string(virtualPath), model->parts.size(),
              asset.materials.size(), asset.images.size());
    return model;
}

} // namespace sim::render
