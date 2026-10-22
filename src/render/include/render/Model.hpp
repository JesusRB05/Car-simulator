// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Turns a glTF document into GPU resources: one `render::Mesh` per primitive
// plus the `render::Material` the shaders want, with the decoded images uploaded
// as mipmapped textures. The node graph, the material list and the `extras` stay
// available so the caller can rig the model (wheels, collision) and cache it.
//
// This is the single upload path for every imported asset - the car and the
// circuit use it - so the renderer owns it instead of the app.

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "asset/Gltf.hpp"
#include "render/Mesh.hpp"
#include "render/Scene.hpp"

namespace sim::vfs {
class VirtualFileSystem;
} // namespace sim::vfs

namespace sim::rhi {
class Device;
} // namespace sim::rhi

namespace sim::render {

// A texture the caller substitutes for one of a material's maps while the model
// is uploaded. Skins ("liveries") use this: they are not part of the glTF, they
// just replace the paint atlas.
struct MaterialOverride {
    // Material to retarget; empty applies the replacement to every material that
    // has the slot.
    std::string material;
    // "base_color" (aliases "body", "albedo") or "emissive".
    std::string slot;
    // Virtual path of the replacement image (PNG/JPEG).
    std::string file;
};

// One drawable piece of a model: a primitive's mesh, its material and the node
// that carries it.
struct ModelPart {
    std::shared_ptr<Mesh> mesh;
    Material material;
    int node = -1;          // index into Model::source.nodes
    int materialIndex = -1; // index into Model::source.materials
};

struct Model {
    // The document, with the vertex/index and pixel data released once uploaded.
    // Nodes, materials, texture descriptors, bounds and extras remain.
    asset::GltfAsset source;
    std::vector<ModelPart> parts;

    // The node's mesh in its own space (the accumulated root-to-node transform),
    // or identity when the index is out of range.
    glm::mat4 nodeWorld(int node) const;
    const asset::Node* node(std::string_view name) const { return source.findNode(name); }
};

// Loads `virtualPath` through the VFS and uploads everything it needs. Returns
// nullptr when the document cannot be read or validated; `warnings` collects the
// recoverable issues (a missing skin image, an image that failed to decode).
std::shared_ptr<Model> loadModel(rhi::Device& device, const vfs::VirtualFileSystem& vfs, std::string_view virtualPath,
                                 const std::vector<MaterialOverride>& overrides = {},
                                 std::vector<std::string>* warnings = nullptr);

// Translates a glTF material into the renderer's material. Exposed for tests.
Material toRenderMaterial(const asset::Material& source);

// Uploads one mipmapped texture per image the materials reference and returns
// the render materials. `asset.images` keeps its pixels; the textures are owned
// by the returned materials. `loadModel` and the circuit importer share this, so
// a texture is uploaded through exactly one path.
std::vector<Material> uploadMaterials(rhi::Device& device, const asset::GltfAsset& asset);

} // namespace sim::render
