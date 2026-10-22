// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/MeshLods.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

#include <meshoptimizer.h>

namespace sim::app::drive {
namespace {

// The bits of a float position, with negative zero folded to zero: the identity
// meshoptimizer itself uses to decide whether two vertices are at the same
// place.
std::uint32_t positionBits(float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits == 0x80000000u ? 0u : bits;
}

struct PositionKey {
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    std::uint32_t z = 0;

    bool operator==(const PositionKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct PositionKeyHash {
    std::size_t operator()(const PositionKey& key) const {
        std::uint64_t hash = key.x * 0x9E3779B97F4A7C15ull;
        hash ^= key.y + 0x9E3779B97F4A7C15ull + (hash << 6) + (hash >> 2);
        hash ^= key.z + 0x9E3779B97F4A7C15ull + (hash << 6) + (hash >> 2);
        return static_cast<std::size_t>(hash);
    }
};

// The geometric border of a mesh, computed on positions rather than on indices.
//
// glTF splits a vertex into one copy per texture seam, so the indices alone see
// a seam as an open border: `meshopt_SimplifyLockBorder` classifies those
// vertices as seams and happily collapses them. Grouping by position first
// recovers the real surface, and the seam edges pair up instead of counting as
// borders. The vertices on the surviving border are then locked explicitly.
struct BorderAnalysis {
    // One canonical vertex id per input vertex.
    std::vector<std::uint32_t> canonical;
    std::size_t canonicalCount = 0;
    std::size_t borderCount = 0; // canonical vertices on the border
    // `meshopt_SimplifyVertex_Lock` for every input vertex whose position is on
    // the border: collapsing any of them would move the border a neighbouring
    // tile shares.
    std::vector<unsigned char> lock;
};

BorderAnalysis analyseBorder(const render::MeshData& data) {
    BorderAnalysis analysis;
    analysis.canonical.resize(data.vertices.size());

    std::unordered_map<PositionKey, std::uint32_t, PositionKeyHash> groups;
    groups.reserve(data.vertices.size());
    for (std::size_t i = 0; i < data.vertices.size(); ++i) {
        const glm::vec3& position = data.vertices[i].position;
        const PositionKey key{positionBits(position.x), positionBits(position.y), positionBits(position.z)};
        const auto [entry, inserted] = groups.try_emplace(key, static_cast<std::uint32_t>(analysis.canonicalCount));
        if (inserted) {
            ++analysis.canonicalCount;
        }
        analysis.canonical[i] = entry->second;
    }

    // An edge of the welded geometry that belongs to a single triangle has no
    // paired triangle: it is on the border.
    std::unordered_map<std::uint64_t, int> edges;
    edges.reserve(data.indices.size());
    for (std::size_t triangle = 0; triangle + 2 < data.indices.size(); triangle += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            const std::uint32_t a = analysis.canonical[data.indices[triangle + static_cast<std::size_t>(corner)]];
            const std::uint32_t b =
                analysis.canonical[data.indices[triangle + static_cast<std::size_t>((corner + 1) % 3)]];
            ++edges[(static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b)];
        }
    }
    std::vector<bool> onBorder(analysis.canonicalCount, false);
    for (const auto& [edge, count] : edges) {
        if (count != 1) {
            continue;
        }
        for (const std::uint32_t canonical :
             {static_cast<std::uint32_t>(edge >> 32), static_cast<std::uint32_t>(edge & 0xffffffffu)}) {
            if (!onBorder[canonical]) {
                onBorder[canonical] = true;
                ++analysis.borderCount;
            }
        }
    }

    analysis.lock.assign(data.vertices.size(), 0);
    for (std::size_t i = 0; i < data.vertices.size(); ++i) {
        if (onBorder[analysis.canonical[i]]) {
            analysis.lock[i] = meshopt_SimplifyVertex_Lock;
        }
    }
    return analysis;
}

} // namespace

std::vector<std::vector<std::uint32_t>> buildMeshLods(const render::MeshData& data, const LodOptions& options) {
    std::vector<std::vector<std::uint32_t>> levels;
    if (data.vertices.empty() || data.indices.size() / 3 < options.minTriangles) {
        return levels;
    }

    BorderAnalysis border;
    if (options.lockBorder) {
        border = analyseBorder(data);
        // A thin strip (an armco rail, a painted line, a fence) is almost all
        // border: there is no surface to simplify and a full lock would leave
        // the simplifier nothing to do.
        if (static_cast<float>(border.borderCount) >
            options.maxBorderRatio * static_cast<float>(border.canonicalCount)) {
            return levels;
        }
    }

    // `destination` must hold the worst case, which is the input index count.
    std::vector<std::uint32_t> simplified(data.indices.size());
    const float* positions = &data.vertices.front().position.x;
    const unsigned char* lock = options.lockBorder && border.borderCount > 0 ? border.lock.data() : nullptr;
    std::size_t previous = data.indices.size();

    for (const float ratio : options.ratios) {
        const std::size_t target = std::max<std::size_t>(
            3, static_cast<std::size_t>(static_cast<double>(data.indices.size()) * ratio) / 3 * 3);
        float error = 0.0f;
        // Position-only simplification with an explicit per-vertex lock. The
        // alternative (`meshopt_SimplifyLockBorder`) only understands index
        // borders and would let the textured seams of a glTF move.
        const std::size_t count = meshopt_simplifyWithAttributes(
            simplified.data(), data.indices.data(), data.indices.size(), positions, data.vertices.size(),
            sizeof(render::Vertex), nullptr, 0, nullptr, 0, lock, target, options.targetError, 0, &error);
        // The error budget or the locked border may stop the simplifier early:
        // only keep a level that is meaningfully coarser than the previous one.
        if (count < 3 || static_cast<float>(count) > static_cast<float>(previous) * options.minSaving) {
            continue;
        }
        levels.emplace_back(simplified.begin(), simplified.begin() + static_cast<std::ptrdiff_t>(count));
        previous = count;
    }
    return levels;
}

} // namespace sim::app::drive
