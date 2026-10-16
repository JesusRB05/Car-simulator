// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "render/Frustum.hpp"
#include "rhi/Device.hpp"
#include "rhi/Resources.hpp"

namespace sim::render {

struct Vertex {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec2 uv{0.0f};
};

// CPU-side geometry. Triangles are wound counter-clockwise when seen from
// outside, i.e. (p1 - p0) x (p2 - p0) points along the vertex normals.
struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
};

// Device-free generators, so the winding and normals can be unit-tested.
MeshData cubeData();
MeshData sphereData();
MeshData cylinderData();
MeshData groundPlaneData(float halfSize);

// Local-space bounds of a vertex list. Device-free as well: the culling volume
// the renderer uses comes from here, so a wrong bound would hide geometry.
Aabb meshBounds(const std::vector<Vertex>& vertices);

// A CPU-authored mesh uploaded into immutable vertex and index buffers.
class Mesh {
public:
    Mesh(rhi::Device& device, const std::vector<Vertex>& vertices, const std::vector<std::uint32_t>& indices);

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) = default;
    Mesh& operator=(Mesh&&) = default;

    const rhi::Buffer& vertexBuffer() const { return vertexBuffer_; }
    const rhi::Buffer& indexBuffer() const { return indexBuffer_; }
    std::uint32_t indexCount() const { return indexCount_; }

    // Local-space bounds of the vertices, computed once on upload. The renderer
    // transforms them into a culling volume without touching the buffer; an empty
    // mesh keeps a zero-size box at the origin (it draws nothing either way).
    const glm::vec3& boundsMin() const { return boundsMin_; }
    const glm::vec3& boundsMax() const { return boundsMax_; }

    static std::shared_ptr<Mesh> makeCube(rhi::Device& device);
    static std::shared_ptr<Mesh> makeSphere(rhi::Device& device);
    static std::shared_ptr<Mesh> makeCylinder(rhi::Device& device);
    static std::shared_ptr<Mesh> makeGroundPlane(rhi::Device& device, float halfSize);

private:
    rhi::Buffer vertexBuffer_;
    rhi::Buffer indexBuffer_;
    std::uint32_t indexCount_ = 0;
    glm::vec3 boundsMin_{0.0f};
    glm::vec3 boundsMax_{0.0f};
};

} // namespace sim::render
