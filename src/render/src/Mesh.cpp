// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Mesh.hpp"

#include <cmath>
#include <utility>

namespace sim::render {
namespace {

rhi::BufferUsage vertexUsage() {
    return rhi::BufferUsage::Vertex | rhi::BufferUsage::TransferDst;
}

rhi::BufferUsage indexUsage() {
    return rhi::BufferUsage::Index | rhi::BufferUsage::TransferDst;
}

} // namespace

Mesh::Mesh(rhi::Device& device, const std::vector<Vertex>& vertices, const std::vector<std::uint32_t>& indices)
    : vertexBuffer_(device, vertices.data(), vertices.size() * sizeof(Vertex), vertexUsage(),
                    rhi::MemoryUsage::CpuToGpu),
      indexBuffer_(device, indices.data(), indices.size() * sizeof(std::uint32_t), indexUsage(),
                   rhi::MemoryUsage::CpuToGpu),
      indexCount_(static_cast<std::uint32_t>(indices.size())) {}

MeshData cubeData() {
    const glm::vec3 faceNormals[6] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
    const glm::vec3 corners[8] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                                  {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
    const int faces[6][4] = {{4, 5, 6, 7}, {1, 0, 3, 2}, {5, 1, 2, 6}, {0, 4, 7, 3}, {3, 7, 6, 2}, {0, 1, 5, 4}};
    const glm::vec2 faceUVs[4] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};

    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    for (int face = 0; face < 6; ++face) {
        const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
        for (int corner = 0; corner < 4; ++corner) {
            Vertex vertex;
            vertex.position = corners[faces[face][corner]] * 0.5f;
            vertex.normal = faceNormals[face];
            vertex.uv = faceUVs[corner];
            vertices.push_back(vertex);
        }
        indices.push_back(base + 0);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
        indices.push_back(base + 0);
        indices.push_back(base + 2);
        indices.push_back(base + 3);
    }

    return {std::move(vertices), std::move(indices)};
}

MeshData sphereData() {
    // Unit UV sphere (radius 1, centred at the origin). Positions equal their
    // normals on a unit sphere, so the Lambert shading is smooth.
    const int slices = 24;
    const int stacks = 14;
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    for (int stack = 0; stack <= stacks; ++stack) {
        const float phi = static_cast<float>(stack) * 3.14159265358979f / static_cast<float>(stacks);
        // Exact poles: sin(pi) in float is a tiny negative number that would
        // fold the bottom-pole triangles inside out.
        const bool pole = stack == 0 || stack == stacks;
        const float sinPhi = pole ? 0.0f : std::sin(phi);
        const float cosPhi = stack == 0 ? 1.0f : (stack == stacks ? -1.0f : std::cos(phi));
        for (int slice = 0; slice <= slices; ++slice) {
            const float theta = static_cast<float>(slice) * 6.28318530717959f / static_cast<float>(slices);
            Vertex vertex;
            vertex.position = {sinPhi * std::cos(theta), cosPhi, sinPhi * std::sin(theta)};
            vertex.normal = vertex.position;
            vertex.uv = {static_cast<float>(slice) / static_cast<float>(slices),
                         static_cast<float>(stack) / static_cast<float>(stacks)};
            vertices.push_back(vertex);
        }
    }
    for (int stack = 0; stack < stacks; ++stack) {
        for (int slice = 0; slice < slices; ++slice) {
            const std::uint32_t a =
                static_cast<std::uint32_t>(stack) * (slices + 1) + static_cast<std::uint32_t>(slice);
            const std::uint32_t b = a + 1;
            const std::uint32_t c = a + static_cast<std::uint32_t>(slices) + 2;
            const std::uint32_t d = a + static_cast<std::uint32_t>(slices) + 1;
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(d);
        }
    }
    return {std::move(vertices), std::move(indices)};
}

MeshData cylinderData() {
    // Unit cylinder: radius 1, half-height 1, axis +Y. Side normals are radial;
    // both caps point up/down so the Lambert shading reads the disc correctly.
    const int slices = 24;
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;

    // capNormalY = 0 builds a side ring with radial normals; +-1 builds a cap
    // ring whose normals point straight up or down.
    const auto ring = [&](float y, float capNormalY) {
        for (int slice = 0; slice <= slices; ++slice) {
            const float theta = static_cast<float>(slice) * 6.28318530717959f / static_cast<float>(slices);
            Vertex vertex;
            vertex.position = {std::cos(theta), y, std::sin(theta)};
            vertex.normal = capNormalY != 0.0f ? glm::vec3(0.0f, capNormalY, 0.0f)
                                               : glm::vec3(std::cos(theta), 0.0f, std::sin(theta));
            vertex.uv = {static_cast<float>(slice) / static_cast<float>(slices), y * 0.5f + 0.5f};
            vertices.push_back(vertex);
        }
    };
    ring(1.0f, 1.0f);
    ring(-1.0f, -1.0f);
    const std::uint32_t sideOffset = static_cast<std::uint32_t>(vertices.size());
    ring(1.0f, 0.0f);
    ring(-1.0f, 0.0f);

    for (int slice = 0; slice < slices; ++slice) {
        const std::uint32_t a = sideOffset + static_cast<std::uint32_t>(slice);
        const std::uint32_t b = a + 1;
        const std::uint32_t c =
            sideOffset + static_cast<std::uint32_t>(slices) + 1 + static_cast<std::uint32_t>(slice) + 1;
        const std::uint32_t d = sideOffset + static_cast<std::uint32_t>(slices) + 1 + static_cast<std::uint32_t>(slice);
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
        indices.push_back(a);
        indices.push_back(c);
        indices.push_back(d);
    }

    // Top and bottom fans over the cap rings (offsets 0 and slices + 1), so the
    // discs carry flat +-Y normals instead of the radial side normals.
    const std::uint32_t topRing = 0;
    const std::uint32_t bottomRing = static_cast<std::uint32_t>(slices) + 1;
    const std::uint32_t topCenter = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back({{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.5f, 0.5f}});
    for (int slice = 0; slice < slices; ++slice) {
        indices.push_back(topCenter);
        indices.push_back(topRing + static_cast<std::uint32_t>(slice) + 1);
        indices.push_back(topRing + static_cast<std::uint32_t>(slice));
    }
    const std::uint32_t bottomCenter = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back({{0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.5f, 0.5f}});
    for (int slice = 0; slice < slices; ++slice) {
        indices.push_back(bottomCenter);
        indices.push_back(bottomRing + static_cast<std::uint32_t>(slice));
        indices.push_back(bottomRing + static_cast<std::uint32_t>(slice) + 1);
    }

    return {std::move(vertices), std::move(indices)};
}

MeshData groundPlaneData(float halfSize) {
    std::vector<Vertex> vertices = {
        {{-halfSize, 0.0f, -halfSize}, {0, 1, 0}, {0, 0}},
        {{halfSize, 0.0f, -halfSize}, {0, 1, 0}, {1, 0}},
        {{halfSize, 0.0f, halfSize}, {0, 1, 0}, {1, 1}},
        {{-halfSize, 0.0f, halfSize}, {0, 1, 0}, {0, 1}},
    };
    // Counter-clockwise seen from above, so the face normal is +Y.
    std::vector<std::uint32_t> indices = {0, 2, 1, 0, 3, 2};
    return {std::move(vertices), std::move(indices)};
}

std::shared_ptr<Mesh> Mesh::makeCube(rhi::Device& device) {
    const MeshData data = cubeData();
    return std::make_shared<Mesh>(device, data.vertices, data.indices);
}

std::shared_ptr<Mesh> Mesh::makeSphere(rhi::Device& device) {
    const MeshData data = sphereData();
    return std::make_shared<Mesh>(device, data.vertices, data.indices);
}

std::shared_ptr<Mesh> Mesh::makeCylinder(rhi::Device& device) {
    const MeshData data = cylinderData();
    return std::make_shared<Mesh>(device, data.vertices, data.indices);
}

std::shared_ptr<Mesh> Mesh::makeGroundPlane(rhi::Device& device, float halfSize) {
    const MeshData data = groundPlaneData(halfSize);
    return std::make_shared<Mesh>(device, data.vertices, data.indices);
}

} // namespace sim::render
