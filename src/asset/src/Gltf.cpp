// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "asset/Gltf.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
#include <stb_image.h>

#include "vfs/VirtualFileSystem.hpp"

namespace sim::asset {
namespace {

// ---------------------------------------------------------------------------
// Small string helpers
// ---------------------------------------------------------------------------

std::string toString(const char* text) {
    return text != nullptr ? std::string(text) : std::string{};
}

int hexDigit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

std::string percentDecode(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '%' && i + 2 < text.size()) {
            const int high = hexDigit(text[i + 1]);
            const int low = hexDigit(text[i + 2]);
            if (high >= 0 && low >= 0) {
                out.push_back(static_cast<char>(high * 16 + low));
                i += 2;
                continue;
            }
        }
        out.push_back(text[i]);
    }
    return out;
}

// Resolves a glTF URI against the directory that holds the glTF file itself,
// yielding a virtual path the VFS can open.
std::string joinVirtual(std::string_view gltfPath, std::string_view uri) {
    const std::string decoded = percentDecode(uri);
    if (!decoded.empty() && (decoded.front() == '/' || decoded.find(':') != std::string::npos)) {
        return decoded; // absolute or a data:/scheme URI, used verbatim
    }
    std::string dir(gltfPath);
    const std::size_t slash = dir.find_last_of('/');
    dir = slash == std::string::npos ? std::string{} : dir.substr(0, slash + 1);
    return dir + decoded;
}

// ---------------------------------------------------------------------------
// `extras` (the JSON blob the content uses for gameplay metadata)
// ---------------------------------------------------------------------------

std::string extrasJson(const cgltf_data* data, const cgltf_extras& extras) {
    cgltf_size size = 0;
    if (cgltf_copy_extras_json(data, &extras, nullptr, &size) != cgltf_result_success || size <= 1) {
        return {};
    }
    std::string out(size, '\0');
    if (cgltf_copy_extras_json(data, &extras, out.data(), &size) != cgltf_result_success) {
        return {};
    }
    out.resize(std::strlen(out.c_str()));
    return out;
}

// Finds `"key"` then the ':' that follows it, returning the offset just past it.
std::size_t findValue(const std::string& json, std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    std::size_t pos = 0;
    while ((pos = json.find(needle, pos)) != std::string::npos) {
        std::size_t after = pos + needle.size();
        while (after < json.size() && (json[after] == ' ' || json[after] == '\t')) {
            ++after;
        }
        if (after < json.size() && json[after] == ':') {
            ++after;
            while (after < json.size() && (json[after] == ' ' || json[after] == '\t')) {
                ++after;
            }
            return after;
        }
        pos = after;
    }
    return std::string::npos;
}

bool jsonBool(const std::string& json, std::string_view key, bool& out) {
    const std::size_t pos = findValue(json, key);
    if (pos == std::string::npos) {
        return false;
    }
    if (json.compare(pos, 4, "true") == 0) {
        out = true;
        return true;
    }
    if (json.compare(pos, 5, "false") == 0) {
        out = false;
        return true;
    }
    return false;
}

bool jsonNumber(const std::string& json, std::string_view key, double& out) {
    const std::size_t pos = findValue(json, key);
    if (pos == std::string::npos) {
        return false;
    }
    char* end = nullptr;
    const double value = std::strtod(json.c_str() + pos, &end);
    if (end == json.c_str() + pos) {
        return false;
    }
    out = value;
    return true;
}

bool jsonString(const std::string& json, std::string_view key, std::string& out) {
    const std::size_t pos = findValue(json, key);
    if (pos == std::string::npos || pos >= json.size() || json[pos] != '"') {
        return false;
    }
    const std::size_t end = json.find('"', pos + 1);
    if (end == std::string::npos) {
        return false;
    }
    out = json.substr(pos + 1, end - pos - 1);
    return true;
}

// ---------------------------------------------------------------------------
// VFS file callbacks (external buffers and images)
// ---------------------------------------------------------------------------

struct VfsContext {
    const vfs::VirtualFileSystem* vfs = nullptr;
};

cgltf_result vfsFileRead(const cgltf_memory_options*, const cgltf_file_options* fileOptions, const char* path,
                         cgltf_size* size, void** data) {
    const auto* context = static_cast<const VfsContext*>(fileOptions != nullptr ? fileOptions->user_data : nullptr);
    if (context == nullptr || context->vfs == nullptr || path == nullptr) {
        return cgltf_result_invalid_options;
    }
    const std::optional<std::vector<std::uint8_t>> bytes = context->vfs->readBinary(path);
    if (!bytes) {
        return cgltf_result_file_not_found;
    }
    void* out = std::malloc(bytes->empty() ? 1 : bytes->size());
    if (out == nullptr) {
        return cgltf_result_out_of_memory;
    }
    if (!bytes->empty()) {
        std::memcpy(out, bytes->data(), bytes->size());
    }
    *size = bytes->size();
    *data = out;
    return cgltf_result_success;
}

void vfsFileRelease(const cgltf_memory_options*, const cgltf_file_options*, void* data) {
    std::free(data);
}

std::string cgltfError(cgltf_result result) {
    switch (result) {
    case cgltf_result_success:
        return "success";
    case cgltf_result_data_too_short:
        return "the file is truncated";
    case cgltf_result_unknown_format:
        return "unknown format (not glTF or GLB)";
    case cgltf_result_invalid_json:
        return "invalid JSON";
    case cgltf_result_invalid_gltf:
        return "invalid glTF";
    case cgltf_result_invalid_options:
        return "invalid options";
    case cgltf_result_file_not_found:
        return "a referenced file is missing";
    case cgltf_result_io_error:
        return "I/O error";
    case cgltf_result_out_of_memory:
        return "out of memory";
    case cgltf_result_legacy_gltf:
        return "legacy glTF 1.0 is not supported";
    default:
        return "unknown error";
    }
}

// ---------------------------------------------------------------------------
// Image decoding
// ---------------------------------------------------------------------------

bool decodeImage(const vfs::VirtualFileSystem& vfs, const cgltf_options& options, std::string_view gltfPath,
                 const cgltf_image& image, const GltfLoadOptions& loadOptions, Image& out,
                 std::vector<std::string>& warnings) {
    const std::string name = toString(image.name);
    const auto warn = [&](std::string message) {
        warnings.push_back(std::string(gltfPath) + ": image '" + name + "': " + std::move(message));
    };

    const std::uint8_t* bytes = nullptr;
    cgltf_size size = 0;
    std::vector<std::uint8_t> owned;
    void* base64 = nullptr;

    if (image.buffer_view != nullptr) {
        bytes = cgltf_buffer_view_data(image.buffer_view);
        size = image.buffer_view->size;
    } else if (image.uri != nullptr) {
        const std::string uri = image.uri;
        if (uri.rfind("data:", 0) == 0) {
            const std::size_t comma = uri.find(',');
            if (comma == std::string::npos || comma + 1 >= uri.size()) {
                warn("malformed data URI");
                return false;
            }
            const std::string_view payload(uri.data() + comma + 1, uri.size() - comma - 1);
            // Every four base64 characters carry three bytes, minus the padding.
            std::size_t padding = 0;
            for (std::size_t i = payload.size(); i > 0 && payload[i - 1] == '='; --i) {
                ++padding;
            }
            const cgltf_size decoded =
                (payload.size() / 4) * 3 > padding ? static_cast<cgltf_size>((payload.size() / 4) * 3 - padding) : 0;
            if (decoded == 0 ||
                cgltf_load_buffer_base64(&options, decoded, payload.data(), &base64) != cgltf_result_success) {
                warn("cannot decode the base64 image data");
                return false;
            }
            bytes = static_cast<const std::uint8_t*>(base64);
            size = decoded;
        } else if (uri.size() >= 5 && (uri.compare(uri.size() - 5, 5, ".ktx2") == 0)) {
            warn("KTX2 textures are not supported yet");
            return false;
        } else {
            const std::string path = joinVirtual(gltfPath, uri);
            std::optional<std::vector<std::uint8_t>> file = vfs.readBinary(path);
            if (!file) {
                warn("cannot read '" + path + "'");
                return false;
            }
            owned = std::move(*file);
            bytes = owned.data();
            size = owned.size();
        }
    } else {
        warn("no data");
        return false;
    }

    if (bytes == nullptr || size == 0) {
        warn("empty data");
        std::free(base64);
        return false;
    }

    int width = 0;
    int height = 0;
    int channels = 0;
    stbi_uc* pixels = stbi_load_from_memory(bytes, static_cast<int>(size), &width, &height, &channels, 4);
    std::free(base64);
    if (pixels == nullptr) {
        warn("cannot decode the image (unsupported format?)");
        return false;
    }
    if (width <= 0 || height <= 0) {
        stbi_image_free(pixels);
        warn("the image has no size");
        return false;
    }
    if (static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height) > loadOptions.maxImagePixels) {
        stbi_image_free(pixels);
        warn("the image exceeds the pixel ceiling");
        return false;
    }

    out.name = name;
    out.width = static_cast<std::uint32_t>(width);
    out.height = static_cast<std::uint32_t>(height);
    const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u;
    out.pixels.assign(pixels, pixels + count);
    stbi_image_free(pixels);
    return true;
}

// ---------------------------------------------------------------------------
// Accessor unpacking
// ---------------------------------------------------------------------------

bool unpackFloats(const cgltf_accessor* accessor, std::size_t components, std::vector<cgltf_float>& out) {
    if (accessor == nullptr || accessor->count == 0) {
        return false;
    }
    out.assign(static_cast<std::size_t>(accessor->count) * components, 0.0f);
    const cgltf_size written = cgltf_accessor_unpack_floats(accessor, out.data(), out.size());
    return written == out.size();
}

int materialTextureIndex(const cgltf_data* data, const cgltf_texture_view& view) {
    if (view.texture == nullptr) {
        return -1;
    }
    return static_cast<int>(cgltf_texture_index(data, view.texture));
}

void unpackPrimitive(const cgltf_data* data, const cgltf_primitive& primitive, const GltfLoadOptions& options,
                     Primitive& out, std::vector<std::string>& warnings, bool& tooLarge) {
    if (primitive.type != cgltf_primitive_type_triangles) {
        warnings.push_back("primitive type is not TRIANGLES; skipped");
        return;
    }
    const cgltf_accessor* position = cgltf_find_accessor(&primitive, cgltf_attribute_type_position, 0);
    if (position == nullptr) {
        warnings.push_back("primitive has no POSITION; skipped");
        return;
    }

    const std::size_t count = static_cast<std::size_t>(position->count);
    if (count > options.maxVertices) {
        tooLarge = true;
        return;
    }

    std::vector<cgltf_float> positions;
    if (!unpackFloats(position, 3, positions)) {
        warnings.push_back("POSITION accessor cannot be unpacked; skipped");
        return;
    }
    std::vector<cgltf_float> normals;
    if (const cgltf_accessor* normal = cgltf_find_accessor(&primitive, cgltf_attribute_type_normal, 0)) {
        unpackFloats(normal, 3, normals);
    }
    std::vector<cgltf_float> uvs;
    if (const cgltf_accessor* uv = cgltf_find_accessor(&primitive, cgltf_attribute_type_texcoord, 0)) {
        unpackFloats(uv, 2, uvs);
    }
    std::vector<cgltf_float> tangents;
    if (const cgltf_accessor* tangent = cgltf_find_accessor(&primitive, cgltf_attribute_type_tangent, 0)) {
        unpackFloats(tangent, 4, tangents);
    }
    std::vector<cgltf_float> colors;
    if (const cgltf_accessor* color = cgltf_find_accessor(&primitive, cgltf_attribute_type_color, 0)) {
        const std::size_t components = cgltf_num_components(color->type);
        unpackFloats(color, components, colors);
    }

    out.vertices.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        Vertex& vertex = out.vertices[i];
        vertex.position = {positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]};
        if (normals.size() >= (i + 1) * 3) {
            vertex.normal = {normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]};
        }
        if (uvs.size() >= (i + 1) * 2) {
            vertex.uv = {uvs[i * 2 + 0], uvs[i * 2 + 1]};
        }
        if (tangents.size() >= (i + 1) * 4) {
            vertex.tangent = {tangents[i * 4 + 0], tangents[i * 4 + 1], tangents[i * 4 + 2], tangents[i * 4 + 3]};
        }
        if (colors.size() >= (i + 1) * 3) {
            vertex.color = {colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f};
            if (colors.size() >= (i + 1) * 4) {
                vertex.color.a = colors[i * 4 + 3];
            }
        }
    }
    out.hasTangents = tangents.size() >= count * 4;

    if (primitive.indices != nullptr) {
        const cgltf_size indexCount = primitive.indices->count;
        if (static_cast<std::size_t>(indexCount) > options.maxIndices) {
            tooLarge = true;
            return;
        }
        out.indices.resize(indexCount);
        for (cgltf_size i = 0; i < indexCount; ++i) {
            out.indices[i] = static_cast<std::uint32_t>(cgltf_accessor_read_index(primitive.indices, i));
        }
    } else {
        out.indices.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            out.indices[i] = static_cast<std::uint32_t>(i);
        }
    }

    if (primitive.material != nullptr) {
        out.material = static_cast<int>(cgltf_material_index(data, primitive.material));
    }

    if (options.generateTangents && !out.hasTangents && !out.indices.empty()) {
        generateTangents(out.indices, out.vertices);
        out.hasTangents = true;
    }

    out.boundsMin = out.vertices.empty() ? glm::vec3(0.0f) : out.vertices.front().position;
    out.boundsMax = out.boundsMin;
    for (const Vertex& vertex : out.vertices) {
        out.boundsMin = glm::min(out.boundsMin, vertex.position);
        out.boundsMax = glm::max(out.boundsMax, vertex.position);
    }
}

void readNodeExtras(const cgltf_data* data, const cgltf_node& node, Node& out) {
    const std::string json = extrasJson(data, node.extras);
    if (json.empty()) {
        return;
    }
    out.hasExtras = true;
    out.extras = json;

    bool boolean = false;
    if (jsonBool(json, "collision", boolean)) {
        out.hasCollision = true;
        out.collision = boolean;
    }
    double number = 0.0;
    if (jsonNumber(json, "friction", number)) {
        out.friction = static_cast<float>(number);
    }
    if (jsonNumber(json, "radius", number)) {
        out.radius = static_cast<float>(number);
    }
    if (jsonNumber(json, "width", number)) {
        out.width = static_cast<float>(number);
    }
    if (jsonBool(json, "steered", boolean)) {
        out.steered = boolean;
    }
    std::string text;
    if (jsonString(json, "surface", text)) {
        out.hasSurface = true;
        out.surface = std::move(text);
    }
}

void readMaterialExtras(const cgltf_data* data, const cgltf_material& material, Material& out) {
    const std::string json = extrasJson(data, material.extras);
    if (json.empty()) {
        return;
    }
    out.extras = json;
    double number = 0.0;
    if (jsonNumber(json, "friction", number)) {
        out.friction = static_cast<float>(number);
    }
    std::string text;
    if (jsonString(json, "surface", text)) {
        out.surface = std::move(text);
    }
}

} // namespace

const Node* GltfAsset::findNode(std::string_view name) const {
    for (const Node& node : nodes) {
        if (node.name == name) {
            return &node;
        }
    }
    return nullptr;
}

void generateTangents(const std::vector<std::uint32_t>& indices, std::vector<Vertex>& vertices) {
    if (vertices.empty()) {
        return;
    }
    std::vector<glm::dvec3> tangents(vertices.size(), glm::dvec3(0.0));
    std::vector<glm::dvec3> bitangents(vertices.size(), glm::dvec3(0.0));

    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        const std::uint32_t i0 = indices[i + 0];
        const std::uint32_t i1 = indices[i + 1];
        const std::uint32_t i2 = indices[i + 2];
        if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) {
            continue;
        }
        const Vertex& v0 = vertices[i0];
        const Vertex& v1 = vertices[i1];
        const Vertex& v2 = vertices[i2];
        const glm::dvec3 edge1 = glm::dvec3(v1.position) - glm::dvec3(v0.position);
        const glm::dvec3 edge2 = glm::dvec3(v2.position) - glm::dvec3(v0.position);
        const glm::dvec2 deltaUV1 = glm::dvec2(v1.uv) - glm::dvec2(v0.uv);
        const glm::dvec2 deltaUV2 = glm::dvec2(v2.uv) - glm::dvec2(v0.uv);
        const double determinant =
            static_cast<double>(deltaUV1.x) * deltaUV2.y - static_cast<double>(deltaUV2.x) * deltaUV1.y;
        if (std::abs(determinant) < 1.0e-20) {
            continue;
        }
        const double inverse = 1.0 / determinant;
        const glm::dvec3 tangent =
            (edge1 * static_cast<double>(deltaUV2.y) - edge2 * static_cast<double>(deltaUV1.y)) * inverse;
        const glm::dvec3 bitangent =
            (edge2 * static_cast<double>(deltaUV1.x) - edge1 * static_cast<double>(deltaUV2.x)) * inverse;
        for (std::uint32_t index : {i0, i1, i2}) {
            tangents[index] += tangent;
            bitangents[index] += bitangent;
        }
    }

    for (std::size_t i = 0; i < vertices.size(); ++i) {
        Vertex& vertex = vertices[i];
        const glm::dvec3 normal = glm::dvec3(glm::normalize(vertex.normal));
        glm::dvec3 tangent = tangents[i];
        // Gram-Schmidt against the normal so the basis is orthogonal.
        tangent -= normal * glm::dot(normal, tangent);
        double length = glm::length(tangent);
        if (length < 1.0e-8) {
            // Degenerate UVs: pick any axis not parallel to the normal.
            const glm::dvec3 axis = std::abs(normal.y) < 0.9 ? glm::dvec3(0.0, 1.0, 0.0) : glm::dvec3(1.0, 0.0, 0.0);
            tangent = glm::cross(axis, normal);
            length = glm::length(tangent);
            if (length < 1.0e-8) {
                vertex.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
                continue;
            }
        }
        const glm::dvec3 ortho = tangent / length;
        // Handedness: sign of dot(cross(normal, tangent), bitangent).
        const double handedness = glm::dot(glm::cross(normal, ortho), bitangents[i]) < 0.0 ? -1.0 : 1.0;
        vertex.tangent = glm::vec4(glm::vec3(ortho), static_cast<float>(handedness));
    }
}

GltfLoadResult loadGltf(const vfs::VirtualFileSystem& vfs, std::string_view virtualPath,
                        const GltfLoadOptions& options) {
    GltfLoadResult result;
    const std::string path(virtualPath);

    const std::optional<std::vector<std::uint8_t>> file = vfs.readBinary(path);
    if (!file) {
        result.errors.push_back(path + ": cannot read the file");
        return result;
    }
    if (file->empty()) {
        result.errors.push_back(path + ": the file is empty");
        return result;
    }

    VfsContext context{&vfs};
    cgltf_options cgltfOptions{};
    cgltfOptions.type = cgltf_file_type_invalid; // auto-detect glTF vs GLB
    cgltfOptions.file.read = &vfsFileRead;
    cgltfOptions.file.release = &vfsFileRelease;
    cgltfOptions.file.user_data = &context;

    cgltf_data* data = nullptr;
    cgltf_result status = cgltf_parse(&cgltfOptions, file->data(), file->size(), &data);
    if (status != cgltf_result_success) {
        result.errors.push_back(path + ": " + cgltfError(status));
        return result;
    }
    const std::unique_ptr<cgltf_data, void (*)(cgltf_data*)> guard(data, &cgltf_free);

    status = cgltf_load_buffers(&cgltfOptions, data, path.c_str());
    if (status != cgltf_result_success) {
        result.errors.push_back(path + ": cannot load the buffers: " + cgltfError(status));
        return result;
    }
    status = cgltf_validate(data);
    if (status != cgltf_result_success) {
        result.errors.push_back(path + ": the glTF is invalid: " + cgltfError(status));
        return result;
    }

    if (data->nodes_count > options.maxNodes) {
        result.errors.push_back(path + ": more than " + std::to_string(options.maxNodes) + " nodes");
        return result;
    }

    GltfAsset asset;
    asset.generator = toString(data->asset.generator);

    // --- Nodes --------------------------------------------------------------
    asset.nodes.resize(data->nodes_count);
    for (cgltf_size i = 0; i < data->nodes_count; ++i) {
        const cgltf_node& node = data->nodes[i];
        Node& out = asset.nodes[i];
        out.name = toString(node.name);
        if (node.mesh != nullptr) {
            out.mesh = static_cast<int>(cgltf_mesh_index(data, node.mesh));
        }
        if (node.parent != nullptr) {
            out.parent = static_cast<int>(cgltf_node_index(data, node.parent));
        }
        out.children.reserve(node.children_count);
        for (cgltf_size c = 0; c < node.children_count; ++c) {
            out.children.push_back(static_cast<int>(cgltf_node_index(data, node.children[c])));
        }
        if (node.has_matrix) {
            std::memcpy(&out.local, node.matrix, sizeof(float) * 16);
        } else {
            cgltf_float matrix[16];
            cgltf_node_transform_local(&node, matrix);
            std::memcpy(&out.local, matrix, sizeof(float) * 16);
        }
        cgltf_float world[16];
        cgltf_node_transform_world(&node, world);
        std::memcpy(&out.world, world, sizeof(float) * 16);
        readNodeExtras(data, node, out);
    }
    const auto addRoot = [&asset](cgltf_size index) {
        if (index >= asset.nodes.size()) {
            return;
        }
        const int value = static_cast<int>(index);
        if (std::find(asset.roots.begin(), asset.roots.end(), value) == asset.roots.end()) {
            asset.roots.push_back(value);
        }
    };
    if (data->scene != nullptr) {
        for (cgltf_size i = 0; i < data->scene->nodes_count; ++i) {
            addRoot(cgltf_node_index(data, data->scene->nodes[i]));
        }
    } else if (data->scenes_count > 0) {
        for (cgltf_size i = 0; i < data->scenes[0].nodes_count; ++i) {
            addRoot(cgltf_node_index(data, data->scenes[0].nodes[i]));
        }
    }
    if (asset.roots.empty()) {
        for (cgltf_size i = 0; i < data->nodes_count; ++i) {
            if (data->nodes[i].parent == nullptr) {
                addRoot(i);
            }
        }
    }

    // --- Meshes -------------------------------------------------------------
    std::uint32_t primitiveCount = 0;
    for (cgltf_size i = 0; i < data->meshes_count; ++i) {
        const cgltf_mesh& mesh = data->meshes[i];
        Mesh out;
        out.name = toString(mesh.name);
        out.primitives.reserve(mesh.primitives_count);
        for (cgltf_size p = 0; p < mesh.primitives_count; ++p) {
            if (++primitiveCount > options.maxPrimitives) {
                result.errors.push_back(path + ": more than " + std::to_string(options.maxPrimitives) + " primitives");
                return result;
            }
            Primitive primitive;
            bool tooLarge = false;
            unpackPrimitive(data, mesh.primitives[p], options, primitive, result.warnings, tooLarge);
            if (tooLarge) {
                result.errors.push_back(path + ": the geometry exceeds the size ceiling");
                return result;
            }
            if (!primitive.vertices.empty() && !primitive.indices.empty()) {
                out.primitives.push_back(std::move(primitive));
            }
        }
        asset.meshes.push_back(std::move(out));
    }

    // --- Materials ----------------------------------------------------------
    asset.materials.resize(data->materials_count);
    for (cgltf_size i = 0; i < data->materials_count; ++i) {
        const cgltf_material& material = data->materials[i];
        Material& out = asset.materials[i];
        out.name = toString(material.name);
        if (material.has_pbr_metallic_roughness) {
            const cgltf_pbr_metallic_roughness& pbr = material.pbr_metallic_roughness;
            out.baseColorFactor = {pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2],
                                   pbr.base_color_factor[3]};
            out.metallic = pbr.metallic_factor;
            out.roughness = pbr.roughness_factor;
            out.baseColorTexture = materialTextureIndex(data, pbr.base_color_texture);
            out.metallicRoughnessTexture = materialTextureIndex(data, pbr.metallic_roughness_texture);
        }
        out.normalTexture = materialTextureIndex(data, material.normal_texture);
        out.normalScale = material.normal_texture.scale;
        out.occlusionTexture = materialTextureIndex(data, material.occlusion_texture);
        out.occlusionStrength = material.occlusion_texture.scale;
        out.emissiveTexture = materialTextureIndex(data, material.emissive_texture);
        out.emissiveFactor = {material.emissive_factor[0], material.emissive_factor[1], material.emissive_factor[2]};
        if (material.has_emissive_strength) {
            out.emissiveStrength = material.emissive_strength.emissive_strength;
        }
        if (material.has_clearcoat) {
            out.clearcoatFactor = material.clearcoat.clearcoat_factor;
            out.clearcoatRoughness = material.clearcoat.clearcoat_roughness_factor;
            out.clearcoatTexture = materialTextureIndex(data, material.clearcoat.clearcoat_texture);
            out.clearcoatRoughnessTexture = materialTextureIndex(data, material.clearcoat.clearcoat_roughness_texture);
            out.clearcoatNormalTexture = materialTextureIndex(data, material.clearcoat.clearcoat_normal_texture);
        }
        out.alphaCutoff = material.alpha_cutoff;
        out.doubleSided = material.double_sided != 0;
        switch (material.alpha_mode) {
        case cgltf_alpha_mode_mask:
            out.alphaMode = Material::AlphaMode::Mask;
            break;
        case cgltf_alpha_mode_blend:
            out.alphaMode = Material::AlphaMode::Blend;
            break;
        default:
            out.alphaMode = Material::AlphaMode::Opaque;
            break;
        }
        readMaterialExtras(data, material, out);
    }

    // --- Textures -----------------------------------------------------------
    asset.textures.reserve(data->textures_count);
    for (cgltf_size i = 0; i < data->textures_count; ++i) {
        const cgltf_texture& texture = data->textures[i];
        Texture out;
        out.name = toString(texture.name);
        if (texture.image != nullptr) {
            out.image = static_cast<int>(cgltf_image_index(data, texture.image));
        }
        if (texture.sampler != nullptr) {
            out.repeatU = texture.sampler->wrap_s == cgltf_wrap_mode_repeat;
            out.repeatV = texture.sampler->wrap_t == cgltf_wrap_mode_repeat;
            out.linear = texture.sampler->mag_filter != cgltf_filter_type_nearest;
        }
        asset.textures.push_back(std::move(out));
    }

    // --- Images -------------------------------------------------------------
    if (data->images_count > options.maxImages) {
        result.errors.push_back(path + ": more than " + std::to_string(options.maxImages) + " images");
        return result;
    }
    asset.images.resize(data->images_count);
    for (cgltf_size i = 0; i < data->images_count; ++i) {
        asset.images[i].name = toString(data->images[i].name);
        if (!options.decodeImages) {
            continue;
        }
        Image image;
        if (decodeImage(vfs, cgltfOptions, path, data->images[i], options, image, result.warnings)) {
            asset.images[i] = std::move(image);
        }
    }

    // Mark the colour images: anything sampled by a base-colour or emissive
    // slot is sRGB-encoded, everything else stays linear.
    const auto markSrgb = [&asset](int textureIndex) {
        if (textureIndex < 0 || static_cast<std::size_t>(textureIndex) >= asset.textures.size()) {
            return;
        }
        const int image = asset.textures[static_cast<std::size_t>(textureIndex)].image;
        if (image >= 0 && static_cast<std::size_t>(image) < asset.images.size()) {
            asset.images[static_cast<std::size_t>(image)].srgb = true;
        }
    };
    for (const Material& material : asset.materials) {
        markSrgb(material.baseColorTexture);
        markSrgb(material.emissiveTexture);
        markSrgb(material.clearcoatTexture);
    }

    // --- Bounds -------------------------------------------------------------
    bool first = true;
    for (const Mesh& mesh : asset.meshes) {
        for (const Primitive& primitive : mesh.primitives) {
            if (first) {
                asset.boundsMin = primitive.boundsMin;
                asset.boundsMax = primitive.boundsMax;
                first = false;
            } else {
                asset.boundsMin = glm::min(asset.boundsMin, primitive.boundsMin);
                asset.boundsMax = glm::max(asset.boundsMax, primitive.boundsMax);
            }
        }
    }
    if (first) {
        asset.boundsMin = glm::vec3(0.0f);
        asset.boundsMax = glm::vec3(0.0f);
    }

    result.asset = std::move(asset);
    return result;
}

} // namespace sim::asset
