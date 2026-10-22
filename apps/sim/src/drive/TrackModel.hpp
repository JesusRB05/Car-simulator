// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// A circuit imported from its glTF: the drawable geometry (split into tiles so a
// large terrain is not one unbounded mesh) and the drivable collision triangles
// with their surface friction, taken from the nodes whose `extras.collision` is
// true and the materials whose `extras.surface` names a surface.
//
// Device-free: the import rules (world transforms, tiling, what counts as
// ground) are unit-tested against the shipped `sierra_verde` model without a
// Vulkan device. `TrackVisual` uploads the result.

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "asset/Gltf.hpp"
#include "drive/Track.hpp"
#include "physics/TriangleMesh.hpp"
#include "physics/TriangleMeshBvh.hpp"
#include "render/Mesh.hpp"
#include "vehicle/GroundQuery.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::app::drive {

// One uploaded drawable: a primitive, or a spatial slice of a large one.
struct TrackTile {
    render::MeshData data;
    int material = -1; // index into TrackModel::source.materials
};

struct TrackModel {
    // Nodes, materials, textures and images stay available; the per-primitive
    // geometry is cleared once it has been tiled (the tiles own it).
    asset::GltfAsset source;
    std::vector<TrackTile> tiles;

    // The drivable surface as one triangle soup in world space, plus the peak
    // friction of each triangle (parallel to `collision.indices` / 3).
    physics::TriangleMesh collision;
    std::vector<float> triangleFriction;

    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};

    std::vector<std::string> warnings;
};

struct TrackModelOptions {
    // Primitives above `tileThreshold` triangles are split on a `tileSize`
    // metre grid over XZ, so a 100k-triangle terrain becomes ~130 drawables and
    // the renderer can cull them independently.
    float tileSize = 48.0f;
    std::size_t tileThreshold = 8192;
};

// Reads and imports `path`. Returns false and fills `error` when the file is
// missing or the glTF is malformed.
bool loadTrackModel(const vfs::VirtualFileSystem& vfs, std::string_view path, const TrackModelOptions& options,
                    TrackModel& out, std::string& error);

// The drivable mesh of a circuit, queried by a vertical ray. `friction` is the
// peak friction of the triangle that was hit.
class TrackSurfaceQuery {
public:
    TrackSurfaceQuery(physics::TriangleMesh mesh, std::vector<float> triangleFriction);

    bool sample(physics::Real x, physics::Real z, physics::Real& height, physics::Vec3& normal,
                physics::Real& friction) const;

    const physics::TriangleMeshBvh& bvh() const { return bvh_; }
    std::size_t triangleCount() const { return bvh_.triangleCount(); }

private:
    physics::TriangleMeshBvh bvh_;
    std::vector<float> friction_;
    physics::Real top_ = 0.0;
    physics::Real bottom_ = 0.0;
};

// Adapts a TrackSurfaceQuery to the vehicle's ground query. Outside the mapped
// surface it falls back to another query (the analytic height field), so a car
// that leaves the world is still supported (and the fallback is documented, not
// silent).
class TrackSurfaceGroundQuery final : public vehicle::IGroundQuery {
public:
    TrackSurfaceGroundQuery(const TrackSurfaceQuery& surface, const vehicle::IGroundQuery& fallback)
        : surface_(&surface), fallback_(&fallback) {}

    vehicle::GroundSample sample(physics::Real x, physics::Real z) const override;

private:
    const TrackSurfaceQuery* surface_;
    const vehicle::IGroundQuery* fallback_;
};

} // namespace sim::app::drive
