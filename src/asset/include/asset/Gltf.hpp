// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Device-free glTF 2.0 loading. The loader reads .glb/.gltf through the virtual
// file system, validates the document, resolves the node hierarchy, unpacks
// geometry (positions, normals, UVs, colours, tangents), decodes the embedded or
// external images and reads the material extensions the content uses
// (KHR_materials_clearcoat, KHR_materials_emissive_strength) plus the `extras`
// the game interprets (collision flags, surfaces, wheel pivot metadata).
//
// Nothing here touches Vulkan: the caller turns the result into GPU resources.
// This is what keeps the format parsing unit-testable without a device.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::asset {

// A decoded image, always expanded to 8-bit RGBA, top row first. `srgb` marks
// the images that carry colour data (base colour, emissive) and must be sampled
// through an sRGB view; the rest (normal, metallic-roughness, occlusion) stay
// linear.
struct Image {
    std::string name;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    bool srgb = false;
    std::vector<std::uint8_t> pixels; // RGBA8, width * height * 4 bytes
};

// A glTF texture: which image it samples and how.
struct Texture {
    std::string name;
    int image = -1; // index into GltfAsset::images
    bool repeatU = true;
    bool repeatV = true;
    bool linear = true; // linear filtering (mipmapped)
};

// A PBR metallic-roughness material plus the extensions/extras the content
// uses. Texture indices point into GltfAsset::textures (not images), -1 = none.
struct Material {
    enum class AlphaMode {
        Opaque,
        Mask,
        Blend,
    };

    std::string name;
    glm::vec4 baseColorFactor{1.0f};
    float metallic = 1.0f;
    float roughness = 1.0f;
    glm::vec3 emissiveFactor{0.0f};
    float emissiveStrength = 1.0f;
    float normalScale = 1.0f;
    float occlusionStrength = 1.0f;
    float alphaCutoff = 0.5f;
    // KHR_materials_clearcoat (0 disables it).
    float clearcoatFactor = 0.0f;
    float clearcoatRoughness = 0.03f;
    float clearcoatNormalScale = 1.0f;
    AlphaMode alphaMode = AlphaMode::Opaque;
    bool doubleSided = false;
    int baseColorTexture = -1;
    int metallicRoughnessTexture = -1;
    int normalTexture = -1;
    int occlusionTexture = -1;
    int emissiveTexture = -1;
    int clearcoatTexture = -1;
    int clearcoatRoughnessTexture = -1;
    int clearcoatNormalTexture = -1;
    // Gameplay extras (track surfaces and the like).
    std::string surface;
    float friction = 1.0f;
    // Raw JSON of the material's `extras`, empty when there is none.
    std::string extras;
};

// One vertex of the unpacked geometry. Position/normal/uv are what the base
// content carries; `tangent` (xyz unit, w = bitangent handedness) and `color`
// are filled in (tangents are generated when the file omits them).
struct Vertex {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec2 uv{0.0f};
    glm::vec4 tangent{0.0f};
    glm::vec4 color{1.0f};
};

struct Primitive {
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    int material = -1; // index into GltfAsset::materials
    bool hasTangents = false;
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};
};

struct Mesh {
    std::string name;
    std::vector<Primitive> primitives;
};

// A node of the scene graph. `world` is the accumulated root-to-node transform
// (column-major, uploaded verbatim like every other matrix in the engine).
struct Node {
    std::string name;
    int mesh = -1; // index into GltfAsset::meshes
    int parent = -1;
    std::vector<int> children;
    glm::mat4 local{1.0f};
    glm::mat4 world{1.0f};
    // `extras` the game interprets (see the content READMEs).
    bool hasCollision = false; // extras.collision present
    bool collision = false;    // its value
    bool hasSurface = false;
    std::string surface;
    float friction = 1.0f;
    float radius = 0.0f; // wheel pivot radius (cars)
    float width = 0.0f;  // wheel width (cars)
    bool steered = false;
    bool hasExtras = false;
    std::string extras; // raw JSON
};

struct GltfAsset {
    std::string generator;
    std::vector<Node> nodes;
    std::vector<int> roots; // indices of the top-level nodes
    std::vector<Mesh> meshes;
    std::vector<Material> materials;
    std::vector<Texture> textures;
    std::vector<Image> images;
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};

    // First node with this exact name, or nullptr.
    const Node* findNode(std::string_view name) const;
};

struct GltfLoadOptions {
    // Hard ceilings so a malformed or hostile file cannot exhaust memory.
    std::uint32_t maxNodes = 200'000;
    std::uint32_t maxPrimitives = 500'000;
    std::uint32_t maxVertices = 40'000'000;
    std::uint32_t maxIndices = 120'000'000;
    std::uint32_t maxImages = 8192;
    std::uint64_t maxImagePixels = 4096ull * 4096ull;
    bool decodeImages = true;
    bool generateTangents = true;
};

struct GltfLoadResult {
    std::optional<GltfAsset> asset;
    std::vector<std::string> errors;
    // Recoverable issues (e.g. an image that failed to decode): the asset still
    // loads, the offending feature is skipped.
    std::vector<std::string> warnings;

    bool ok() const { return asset.has_value(); }
};

// Loads a glTF through the VFS. Never throws; a malformed file yields errors and
// an empty asset. `virtualPath` is the file inside the virtual tree, and any
// external buffers/images are resolved relative to it.
GltfLoadResult loadGltf(const vfs::VirtualFileSystem& vfs, std::string_view virtualPath,
                        const GltfLoadOptions& options = {});

// Generates per-vertex tangents (xyz) and handedness (w) for a primitive from
// its positions, normals and UVs, using the standard per-triangle accumulation
// averaged over shared vertices (the MikkTSpace convention). Vertices whose
// tangent is degenerate fall back to a normal-orthogonal basis. Exposed so the
// maths can be unit-tested without any file.
void generateTangents(const std::vector<std::uint32_t>& indices, std::vector<Vertex>& vertices);

} // namespace sim::asset
