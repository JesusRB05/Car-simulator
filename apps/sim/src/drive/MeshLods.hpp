// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Import-time level of detail, generated with meshoptimizer. The levels are
// index lists into the vertex buffer they were built from, so a chain shares one
// upload of the geometry.
//
// Circuits are split into tiles, and a tile can be drawn at a different level
// than its neighbour, so the tile border is locked: the shared edge keeps its
// exact vertices and triangles and the two tiles cannot crack apart.
//
// Device-free, so the contract (a coarser level, an intact border) is unit
// tested without a Vulkan device.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "render/Mesh.hpp"

namespace sim::app::drive {

struct LodOptions {
    // How many triangles each level keeps relative to the full-detail mesh, in
    // the order they are drawn (coarse to finer). A level that cannot reach its
    // target within `targetError` is dropped, and a level that would not save
    // `minSaving` of the triangles is not emitted either.
    std::vector<float> ratios = {0.5f, 0.25f};
    // Allowed deviation, relative to the mesh extent (0.01 = 1%).
    float targetError = 0.01f;
    // A level must keep at most this fraction of its predecessor's triangles.
    float minSaving = 0.9f;
    // Meshes below this triangle count are not worth a chain.
    std::size_t minTriangles = 512;
    // Keep the border (the edges a neighbouring tile shares) intact. The border
    // is found on positions, so the texture seams that glTF splits a vertex into
    // do not count as borders; those vertices are locked explicitly.
    bool lockBorder = true;
    // A mesh whose vertices are more than this fraction on the border is a thin
    // strip with no surface to simplify; it keeps full detail.
    float maxBorderRatio = 0.5f;
};

// Returns the lower levels of detail of `data`, each an index list into
// `data.vertices`. Empty when the mesh is too small or cannot be simplified
// without exceeding the error budget.
std::vector<std::vector<std::uint32_t>> buildMeshLods(const render::MeshData& data, const LodOptions& options);

} // namespace sim::app::drive
