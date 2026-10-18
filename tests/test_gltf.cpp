// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "asset/Gltf.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace {

namespace fs = std::filesystem;

fs::path sourceDir() {
    return fs::path(SIM_SOURCE_DIR);
}

std::size_t triangleCount(const sim::asset::GltfAsset& asset) {
    std::size_t count = 0;
    for (const sim::asset::Mesh& mesh : asset.meshes) {
        for (const sim::asset::Primitive& primitive : mesh.primitives) {
            count += primitive.indices.size() / 3;
        }
    }
    return count;
}

std::size_t vertexCount(const sim::asset::GltfAsset& asset) {
    std::size_t count = 0;
    for (const sim::asset::Mesh& mesh : asset.meshes) {
        for (const sim::asset::Primitive& primitive : mesh.primitives) {
            count += primitive.vertices.size();
        }
    }
    return count;
}

const sim::asset::Material* findMaterial(const sim::asset::GltfAsset& asset, const std::string& name) {
    for (const sim::asset::Material& material : asset.materials) {
        if (material.name == name) {
            return &material;
        }
    }
    return nullptr;
}

} // namespace

TEST_CASE("asset: the Halvard GT3 model loads with its hierarchy, materials and textures") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));

    const sim::asset::GltfLoadResult result = sim::asset::loadGltf(vfs, "cars/halvard_gt3/model/car.glb");
    for (const std::string& error : result.errors) {
        MESSAGE("car.glb error: " << error);
    }
    for (const std::string& warning : result.warnings) {
        MESSAGE("car.glb warning: " << warning);
    }
    REQUIRE(result.ok());
    const sim::asset::GltfAsset& asset = *result.asset;

    CHECK(asset.findNode("car_root") != nullptr);
    CHECK(asset.roots.size() == 1);
    REQUIRE_FALSE(asset.roots.empty());
    CHECK(asset.nodes[static_cast<std::size_t>(asset.roots.front())].name == "car_root");

    // The wheel pivots carry their radius/width/steered extras.
    const sim::asset::Node* front = asset.findNode("wheel_FL");
    REQUIRE(front != nullptr);
    CHECK(front->hasExtras);
    CHECK(front->radius == doctest::Approx(0.340).epsilon(0.01));
    CHECK(front->steered);
    CHECK(front->children.size() == 2);
    REQUIRE(front->children.size() == 2);
    CHECK(asset.nodes[static_cast<std::size_t>(front->children[0])].name == "wheel_FL_spin");
    CHECK(asset.nodes[static_cast<std::size_t>(front->children[1])].name == "wheel_FL_caliper");
    CHECK(asset.nodes[static_cast<std::size_t>(front->children[0])].parent ==
          static_cast<int>(&*front - asset.nodes.data()));

    const sim::asset::Node* rear = asset.findNode("wheel_RL");
    REQUIRE(rear != nullptr);
    CHECK_FALSE(rear->steered);
    CHECK(rear->radius == doctest::Approx(0.355).epsilon(0.01));

    // Geometry: the README budgets ~31k triangles, each vertex carries a tangent.
    CHECK(triangleCount(asset) > 20000);
    std::size_t tangentVertices = 0;
    std::size_t totalVertices = 0;
    for (const sim::asset::Mesh& mesh : asset.meshes) {
        for (const sim::asset::Primitive& primitive : mesh.primitives) {
            if (primitive.hasTangents) {
                tangentVertices += primitive.vertices.size();
            }
            totalVertices += primitive.vertices.size();
        }
    }
    CHECK(tangentVertices == totalVertices);
    for (const sim::asset::Vertex& vertex : asset.meshes.front().primitives.front().vertices) {
        CHECK(glm::length(glm::vec3(vertex.tangent)) == doctest::Approx(1.0f).epsilon(0.001));
        CHECK(std::abs(std::abs(vertex.tangent.w) - 1.0f) < 1.0e-4f);
    }

    // Materials: the paint is clear-coated, textured and emissive.
    const sim::asset::Material* paint = findMaterial(asset, "paint_livery");
    REQUIRE(paint != nullptr);
    CHECK(paint->clearcoatFactor == doctest::Approx(1.0));
    CHECK(paint->clearcoatRoughness == doctest::Approx(0.04));
    CHECK(paint->emissiveStrength == doctest::Approx(4.0));
    CHECK(paint->baseColorTexture >= 0);
    CHECK(paint->emissiveTexture >= 0);
    CHECK(paint->metallic == doctest::Approx(0.15));

    const sim::asset::Material* glass = findMaterial(asset, "glass");
    REQUIRE(glass != nullptr);
    CHECK(glass->alphaMode == sim::asset::Material::AlphaMode::Blend);
    CHECK(glass->doubleSided);

    const sim::asset::Material* rain = findMaterial(asset, "rain_light");
    REQUIRE(rain != nullptr);
    CHECK(rain->emissiveStrength == doctest::Approx(3.0));

    // Textures and images: the livery atlas decodes and is marked sRGB.
    CHECK(asset.textures.size() >= 3);
    CHECK(asset.images.size() >= 3);
    bool liverySrgb = false;
    for (const sim::asset::Image& image : asset.images) {
        if (image.srgb && !image.pixels.empty() && image.width == 2048 && image.height == 2048) {
            liverySrgb = true;
        }
    }
    CHECK(liverySrgb);
    CHECK(vertexCount(asset) > 20000);
}

TEST_CASE("asset: the Sierra Verde circuit carries collision and surface extras") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));

    const sim::asset::GltfLoadResult result = sim::asset::loadGltf(vfs, "tracks/sierra_verde/model/track.glb");
    for (const std::string& error : result.errors) {
        MESSAGE("track.glb error: " << error);
    }
    REQUIRE(result.ok());
    const sim::asset::GltfAsset& asset = *result.asset;

    CHECK(asset.findNode("track_root") != nullptr);

    // Collision flags match the README table.
    const sim::asset::Node* road = asset.findNode("road");
    REQUIRE(road != nullptr);
    CHECK(road->hasCollision);
    CHECK(road->collision);

    const sim::asset::Node* terrain = asset.findNode("terrain");
    REQUIRE(terrain != nullptr);
    CHECK(terrain->collision);

    const sim::asset::Node* startFinish = asset.findNode("start_finish");
    REQUIRE(startFinish != nullptr);
    CHECK(startFinish->hasCollision);
    CHECK_FALSE(startFinish->collision);
    CHECK(asset.findNode("trees") != nullptr);
    CHECK_FALSE(asset.findNode("trees")->collision);

    // Surface ids and friction live on the materials.
    const sim::asset::Material* asphalt = findMaterial(asset, "asphalt");
    REQUIRE(asphalt != nullptr);
    CHECK(asphalt->surface == "asphalt");
    CHECK(asphalt->friction == doctest::Approx(1.0));
    const sim::asset::Material* gravel = findMaterial(asset, "gravel");
    REQUIRE(gravel != nullptr);
    CHECK(gravel->surface == "gravel");
    CHECK(gravel->friction == doctest::Approx(0.45));

    // The terrain alone is a large tileable mesh; the whole circuit is ~330k tris.
    CHECK(triangleCount(asset) > 100000);
    CHECK_FALSE(asset.images.empty());
}

TEST_CASE("asset: an external-buffer .gltf resolves its buffer through the VFS") {
    namespace stdfs = std::filesystem;
    const stdfs::path dir = stdfs::temp_directory_path() / "sim_asset_external_test";
    std::error_code error;
    stdfs::remove_all(dir, error);
    stdfs::create_directories(dir, error);

    // One triangle whose positions and indices live in a separate .bin file.
    const float positions[9] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    const std::uint16_t indices[3] = {0, 1, 2};
    std::vector<std::uint8_t> binary(sizeof(positions) + sizeof(indices));
    std::memcpy(binary.data(), positions, sizeof(positions));
    std::memcpy(binary.data() + sizeof(positions), indices, sizeof(indices));
    {
        std::ofstream out(dir / "mini.bin", std::ios::binary);
        out.write(reinterpret_cast<const char*>(binary.data()), static_cast<std::streamsize>(binary.size()));
    }
    const std::string json =
        "{\"asset\":{\"version\":\"2.0\"},"
        "\"buffers\":[{\"uri\":\"mini.bin\",\"byteLength\":42}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36,\"target\":34962},"
        "{\"buffer\":0,\"byteOffset\":36,\"byteLength\":6,\"target\":34963}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
        "\"min\":[0,0,0],\"max\":[1,1,0]},{\"bufferView\":1,\"componentType\":5123,\"count\":3,\"type\":\"SCALAR\"}],"
        "\"materials\":[{\"name\":\"flat\",\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.2,0.4,0.6,1]}}],"
        "\"meshes\":[{\"name\":\"tri\",\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1,\"material\":0}]}"
        "],"
        "\"nodes\":[{\"name\":\"root\",\"mesh\":0}],"
        "\"scenes\":[{\"nodes\":[0]}],\"scene\":0}";
    {
        std::ofstream out(dir / "mini.gltf");
        out << json;
    }

    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(dir));
    const sim::asset::GltfLoadResult result = sim::asset::loadGltf(vfs, "mini.gltf");
    for (const std::string& message : result.errors) {
        MESSAGE("mini.gltf error: " << message);
    }
    REQUIRE(result.ok());
    CHECK(triangleCount(*result.asset) == 1);
    CHECK(result.asset->materials.size() == 1);
    CHECK(result.asset->materials.front().name == "flat");
    CHECK(result.asset->nodes.front().name == "root");

    stdfs::remove_all(dir, error);
}

TEST_CASE("asset: the image decode can be skipped without losing the material graph") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));

    sim::asset::GltfLoadOptions options;
    options.decodeImages = false;
    const sim::asset::GltfLoadResult result = sim::asset::loadGltf(vfs, "cars/halvard_gt3/model/car.glb", options);
    REQUIRE(result.ok());
    CHECK_FALSE(result.asset->materials.empty());
    for (const sim::asset::Image& image : result.asset->images) {
        CHECK(image.pixels.empty());
    }
}

TEST_CASE("asset: a missing file and an over-large document are rejected cleanly") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));

    const sim::asset::GltfLoadResult missing = sim::asset::loadGltf(vfs, "cars/does_not_exist/model/car.glb");
    CHECK_FALSE(missing.ok());
    CHECK_FALSE(missing.errors.empty());

    sim::asset::GltfLoadOptions tiny;
    tiny.maxNodes = 2;
    const sim::asset::GltfLoadResult tooBig = sim::asset::loadGltf(vfs, "cars/halvard_gt3/model/car.glb", tiny);
    CHECK_FALSE(tooBig.ok());
    REQUIRE_FALSE(tooBig.errors.empty());
    CHECK(tooBig.errors.front().find("nodes") != std::string::npos);
}

TEST_CASE("asset: generated tangents form an orthogonal unit basis following the UVs") {
    // A quad in the XZ plane, UV u along +X and v along +Z, normal +Y.
    std::vector<sim::asset::Vertex> vertices(4);
    vertices[0].position = {0.0f, 0.0f, 0.0f};
    vertices[1].position = {1.0f, 0.0f, 0.0f};
    vertices[2].position = {1.0f, 0.0f, 1.0f};
    vertices[3].position = {0.0f, 0.0f, 1.0f};
    vertices[0].uv = {0.0f, 0.0f};
    vertices[1].uv = {1.0f, 0.0f};
    vertices[2].uv = {1.0f, 1.0f};
    vertices[3].uv = {0.0f, 1.0f};
    for (sim::asset::Vertex& vertex : vertices) {
        vertex.normal = {0.0f, 1.0f, 0.0f};
    }
    const std::vector<std::uint32_t> indices = {0, 2, 1, 0, 3, 2};

    sim::asset::generateTangents(indices, vertices);
    for (const sim::asset::Vertex& vertex : vertices) {
        const glm::vec3 tangent(vertex.tangent);
        CHECK(glm::length(tangent) == doctest::Approx(1.0f).epsilon(0.001));
        CHECK(glm::dot(tangent, vertex.normal) == doctest::Approx(0.0f).epsilon(0.001));
        // u grows along +X, so the tangent points along +X.
        CHECK(tangent.x == doctest::Approx(1.0f).epsilon(0.01));
        CHECK(std::abs(std::abs(vertex.tangent.w) - 1.0f) < 1.0e-4f);
    }
}
