// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/TrackModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>

#include "core/Log.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app::drive {
namespace {

glm::vec3 transformPoint(const glm::mat4& matrix, const glm::vec3& point) {
    return glm::vec3(matrix * glm::vec4(point, 1.0f));
}

render::Vertex transformVertex(const glm::mat4& world, const glm::mat3& normalMatrix, const asset::Vertex& source) {
    render::Vertex vertex;
    vertex.position = transformPoint(world, source.position);
    const glm::vec3 normal = normalMatrix * source.normal;
    vertex.normal = glm::dot(normal, normal) > 1.0e-12f ? glm::normalize(normal) : glm::vec3(0.0f, 1.0f, 0.0f);
    vertex.uv = source.uv;
    const glm::vec3 tangent = normalMatrix * glm::vec3(source.tangent);
    vertex.tangent =
        glm::vec4(glm::dot(tangent, tangent) > 1.0e-12f ? glm::normalize(tangent) : glm::vec3(0.0f), source.tangent.w);
    vertex.color = source.color;
    return vertex;
}

// The material a primitive draws with, or nullptr.
const asset::Material* materialOf(const asset::GltfAsset& asset, int index) {
    if (index < 0 || static_cast<std::size_t>(index) >= asset.materials.size()) {
        return nullptr;
    }
    return &asset.materials[static_cast<std::size_t>(index)];
}

// A triangle is drivable ground when its material names a surface (`asphalt`,
// `kerb`, `grass`, ...) or its node does. Walls, buildings, stands and trees
// carry no surface, so they never enter the wheel query: a downward ray would
// otherwise land on the footbridge above the road.
bool isGround(const asset::Node& node, const asset::Material* material) {
    return (material != nullptr && !material->surface.empty()) || node.hasSurface;
}

float groundFriction(const asset::Node& node, const asset::Material* material) {
    if (material != nullptr && !material->surface.empty()) {
        return material->friction;
    }
    return node.hasSurface ? node.friction : 1.0f;
}

} // namespace

bool loadTrackModel(const vfs::VirtualFileSystem& vfs, std::string_view path, const TrackModelOptions& options,
                    TrackModel& out, std::string& error) {
    asset::GltfLoadResult loaded = asset::loadGltf(vfs, path);
    out.warnings = loaded.warnings;
    if (!loaded.ok()) {
        error = loaded.errors.empty() ? std::string(path) + ": cannot load the circuit model" : loaded.errors.front();
        return false;
    }
    out.source = std::move(*loaded.asset);
    asset::GltfAsset& asset = out.source;

    bool haveBounds = false;

    for (std::size_t nodeIndex = 0; nodeIndex < asset.nodes.size(); ++nodeIndex) {
        const asset::Node& node = asset.nodes[nodeIndex];
        if (node.mesh < 0 || static_cast<std::size_t>(node.mesh) >= asset.meshes.size()) {
            continue;
        }
        asset::Mesh& mesh = asset.meshes[static_cast<std::size_t>(node.mesh)];
        const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(node.world)));

        for (std::size_t primitiveIndex = 0; primitiveIndex < mesh.primitives.size(); ++primitiveIndex) {
            asset::Primitive& primitive = mesh.primitives[primitiveIndex];
            const std::size_t triangleCount = primitive.indices.size() / 3;
            if (triangleCount == 0 || primitive.vertices.empty()) {
                continue;
            }
            const asset::Material* material = materialOf(asset, primitive.material);

            // World-space vertices, computed once and shared by the drawables
            // and the collision triangles.
            std::vector<render::Vertex> worldVertices(primitive.vertices.size());
            for (std::size_t i = 0; i < primitive.vertices.size(); ++i) {
                worldVertices[i] = transformVertex(node.world, normalMatrix, primitive.vertices[i]);
            }

            // ---- Drawables: one tile, or a grid of tiles for a large mesh ----
            std::vector<render::MeshData> tileData;
            if (triangleCount <= options.tileThreshold) {
                render::MeshData data;
                data.vertices = worldVertices;
                data.indices = primitive.indices;
                tileData.push_back(std::move(data));
            } else {
                glm::vec3 min = worldVertices.front().position;
                glm::vec3 max = min;
                for (const render::Vertex& vertex : worldVertices) {
                    min = glm::min(min, vertex.position);
                    max = glm::max(max, vertex.position);
                }
                const float size = std::max(options.tileSize, 1.0f);
                const int columns = std::max(1, static_cast<int>(std::ceil((max.x - min.x) / size)));
                const int rows = std::max(1, static_cast<int>(std::ceil((max.z - min.z) / size)));
                std::vector<std::vector<std::uint32_t>> trianglesByTile(static_cast<std::size_t>(columns) * rows);
                for (std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
                    const glm::vec3 a = worldVertices[primitive.indices[3 * triangle + 0]].position;
                    const glm::vec3 b = worldVertices[primitive.indices[3 * triangle + 1]].position;
                    const glm::vec3 c = worldVertices[primitive.indices[3 * triangle + 2]].position;
                    const glm::vec3 centre = (a + b + c) / 3.0f;
                    const int column = std::clamp(static_cast<int>((centre.x - min.x) / size), 0, columns - 1);
                    const int row = std::clamp(static_cast<int>((centre.z - min.z) / size), 0, rows - 1);
                    trianglesByTile[static_cast<std::size_t>(row * columns + column)].push_back(
                        static_cast<std::uint32_t>(triangle));
                }
                tileData.resize(trianglesByTile.size());
                for (std::size_t tile = 0; tile < trianglesByTile.size(); ++tile) {
                    render::MeshData& data = tileData[tile];
                    std::unordered_map<std::uint32_t, std::uint32_t> remap;
                    for (const std::uint32_t triangle : trianglesByTile[tile]) {
                        for (int corner = 0; corner < 3; ++corner) {
                            const std::uint32_t source =
                                primitive.indices[3 * triangle + static_cast<std::size_t>(corner)];
                            const auto [it, inserted] =
                                remap.try_emplace(source, static_cast<std::uint32_t>(data.vertices.size()));
                            if (inserted) {
                                data.vertices.push_back(worldVertices[source]);
                            }
                            data.indices.push_back(it->second);
                        }
                    }
                }
            }

            for (render::MeshData& data : tileData) {
                if (data.indices.empty()) {
                    continue;
                }
                TrackTile tile;
                tile.material = primitive.material;
                tile.data = std::move(data);
                for (const render::Vertex& vertex : tile.data.vertices) {
                    if (!haveBounds) {
                        out.boundsMin = vertex.position;
                        out.boundsMax = vertex.position;
                        haveBounds = true;
                    } else {
                        out.boundsMin = glm::min(out.boundsMin, vertex.position);
                        out.boundsMax = glm::max(out.boundsMax, vertex.position);
                    }
                }
                out.tiles.push_back(std::move(tile));
            }

            // ---- Collision ----------------------------------------------------
            if (node.hasCollision && node.collision && isGround(node, material)) {
                const float friction = groundFriction(node, material);
                const auto base = static_cast<std::uint32_t>(out.collision.vertices.size());
                out.collision.vertices.reserve(out.collision.vertices.size() + primitive.indices.size());
                out.collision.indices.reserve(out.collision.indices.size() + primitive.indices.size());
                for (std::size_t i = 0; i < primitive.indices.size(); ++i) {
                    const glm::vec3 position = worldVertices[primitive.indices[i]].position;
                    out.collision.vertices.push_back(physics::Vec3(static_cast<physics::Real>(position.x),
                                                                   static_cast<physics::Real>(position.y),
                                                                   static_cast<physics::Real>(position.z)));
                    out.collision.indices.push_back(base + static_cast<std::uint32_t>(i));
                }
                out.triangleFriction.insert(out.triangleFriction.end(), triangleCount, friction);
            }

            std::vector<asset::Vertex>().swap(primitive.vertices);
            std::vector<std::uint32_t>().swap(primitive.indices);
        }
    }

    if (out.tiles.empty()) {
        error = std::string(path) + ": the circuit model has no drawable geometry";
        return false;
    }
    if (out.collision.indices.empty()) {
        error = std::string(path) + ": the circuit model has no collidable surface "
                                    "(no node with extras.collision = true and a surface material)";
        return false;
    }
    return true;
}

TrackSurfaceQuery::TrackSurfaceQuery(physics::TriangleMesh mesh, std::vector<float> triangleFriction)
    : friction_(std::move(triangleFriction)) {
    if (!mesh.vertices.empty()) {
        top_ = mesh.vertices.front().y;
        bottom_ = top_;
        for (const physics::Vec3& vertex : mesh.vertices) {
            top_ = std::max(top_, vertex.y);
            bottom_ = std::min(bottom_, vertex.y);
        }
    }
    bvh_.build(std::move(mesh));
}

bool TrackSurfaceQuery::sample(physics::Real x, physics::Real z, physics::Real& height, physics::Vec3& normal,
                               physics::Real& friction) const {
    if (bvh_.empty()) {
        return false;
    }
    const physics::Real rayTop = top_ + 50.0;
    const physics::Real rayLength = rayTop - (bottom_ - 50.0);
    const physics::MeshRayHit hit = bvh_.raycast(physics::Vec3(x, rayTop, z), physics::Vec3(0.0, -1.0, 0.0), rayLength);
    if (!hit.hit) {
        return false;
    }
    height = hit.point.y;
    normal = hit.normal;
    friction = hit.triangle < friction_.size() ? static_cast<physics::Real>(friction_[hit.triangle]) : 1.0;
    return true;
}

vehicle::GroundSample TrackSurfaceGroundQuery::sample(physics::Real x, physics::Real z) const {
    physics::Real height = 0.0;
    physics::Vec3 normal(0.0, 1.0, 0.0);
    physics::Real friction = 1.0;
    if (surface_->sample(x, z, height, normal, friction)) {
        vehicle::GroundSample result;
        result.height = height;
        result.normal = normal;
        result.friction = friction;
        result.rollingResistance = 0.015;
        return result;
    }
    return fallback_->sample(x, z);
}

} // namespace sim::app::drive
