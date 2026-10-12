// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/Mesh.hpp"

namespace sim::render {
namespace {

rhi::BufferUsage vertexUsage()
{
    return rhi::BufferUsage::Vertex | rhi::BufferUsage::TransferDst;
}

rhi::BufferUsage indexUsage()
{
    return rhi::BufferUsage::Index | rhi::BufferUsage::TransferDst;
}

} // namespace

Mesh::Mesh(rhi::Device& device, const std::vector<Vertex>& vertices, const std::vector<std::uint32_t>& indices)
    : vertexBuffer_(device, vertices.data(), vertices.size() * sizeof(Vertex), vertexUsage(), rhi::MemoryUsage::CpuToGpu),
      indexBuffer_(device, indices.data(), indices.size() * sizeof(std::uint32_t), indexUsage(), rhi::MemoryUsage::CpuToGpu),
      indexCount_(static_cast<std::uint32_t>(indices.size()))
{
}

std::shared_ptr<Mesh> Mesh::makeCube(rhi::Device& device)
{
    const glm::vec3 faceNormals[6] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
    const glm::vec3 corners[8] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                                  {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
    const int faces[6][4] = {{4, 5, 6, 7}, {1, 0, 3, 2}, {5, 1, 2, 6}, {0, 4, 7, 3}, {3, 7, 6, 2}, {0, 1, 5, 4}};
    const glm::vec2 faceUVs[4] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};

    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    for (int face = 0; face < 6; ++face)
    {
        const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
        for (int corner = 0; corner < 4; ++corner)
        {
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

    return std::make_shared<Mesh>(device, vertices, indices);
}

std::shared_ptr<Mesh> Mesh::makeGroundPlane(rhi::Device& device, float halfSize)
{
    std::vector<Vertex> vertices = {
        {{-halfSize, 0.0f, -halfSize}, {0, 1, 0}, {0, 0}},
        {{halfSize, 0.0f, -halfSize}, {0, 1, 0}, {1, 0}},
        {{halfSize, 0.0f, halfSize}, {0, 1, 0}, {1, 1}},
        {{-halfSize, 0.0f, halfSize}, {0, 1, 0}, {0, 1}},
    };
    std::vector<std::uint32_t> indices = {0, 1, 2, 0, 2, 3};
    return std::make_shared<Mesh>(device, vertices, indices);
}

} // namespace sim::render
