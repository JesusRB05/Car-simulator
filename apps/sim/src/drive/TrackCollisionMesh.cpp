// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/TrackCollisionMesh.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace sim::app::drive {
namespace {

// The terrain grid covers the circuit with a margin; a car that leaves it falls
// back to the analytic surface (see TrackMeshGroundQuery).
constexpr Real kMinX = -200.0;
constexpr Real kMaxX = 320.0;
constexpr Real kMinZ = -380.0;
constexpr Real kMaxZ = 380.0;
constexpr Real kCell = 4.0;

// The vertical ray spans well above and below any surface point, so a query
// never has to know the local height.
constexpr Real kRayTop = 400.0;
constexpr Real kRayBottom = -400.0;

void addTriangle(physics::TriangleMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c) {
    const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(a);
    mesh.vertices.push_back(b);
    mesh.vertices.push_back(c);
    mesh.indices.push_back(base);
    mesh.indices.push_back(base + 1);
    mesh.indices.push_back(base + 2);
}

// A quad a-b-c-d split into two triangles.
void addQuad(physics::TriangleMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    addTriangle(mesh, a, b, c);
    addTriangle(mesh, a, c, d);
}

Vec3 onTerrain(const Track& track, Real x, Real z) {
    return Vec3(x, track.heightAt(x, z), z);
}

} // namespace

TrackCollisionMesh::TrackCollisionMesh(const Track& track) {
    physics::TriangleMesh mesh;
    const std::vector<TrackSample>& samples = track.samples();
    const std::size_t count = samples.size();

    // Terrain: the smooth height field, sampled on a regular grid.
    {
        const int columns = static_cast<int>((kMaxX - kMinX) / kCell);
        const int rows = static_cast<int>((kMaxZ - kMinZ) / kCell);
        for (int row = 0; row < rows; ++row) {
            for (int column = 0; column < columns; ++column) {
                const Real x0 = kMinX + kCell * column;
                const Real x1 = x0 + kCell;
                const Real z0 = kMinZ + kCell * row;
                const Real z1 = z0 + kCell;
                addQuad(mesh, onTerrain(track, x0, z0), onTerrain(track, x1, z0), onTerrain(track, x1, z1),
                        onTerrain(track, x0, z1));
            }
        }
    }

    // Asphalt ribbon: one quad per centreline segment, following the per-sample
    // width and banking (the same road points the renderer draws).
    for (std::size_t i = 0; i < count; ++i) {
        const TrackSample& a = samples[i];
        const TrackSample& b = samples[(i + 1) % count];
        addQuad(mesh, track.roadPoint(a, -a.halfWidth, 0.0), track.roadPoint(a, a.halfWidth, 0.0),
                track.roadPoint(b, b.halfWidth, 0.0), track.roadPoint(b, -b.halfWidth, 0.0));
    }

    // Kerbs: the raised band just outside the asphalt, where a sample carries one.
    for (std::size_t i = 0; i < count; ++i) {
        const TrackSample& a = samples[i];
        if (!a.kerb) {
            continue;
        }
        const TrackSample& b = samples[(i + 1) % count];
        for (const Real side : {-1.0, 1.0}) {
            addQuad(mesh, track.roadPoint(a, side * a.halfWidth, Track::kKerbLift),
                    track.roadPoint(a, side * (a.halfWidth + track.kerbWidth()), Track::kKerbLift),
                    track.roadPoint(b, side * (b.halfWidth + track.kerbWidth()), Track::kKerbLift),
                    track.roadPoint(b, side * b.halfWidth, Track::kKerbLift));
        }
    }

    bvh_.build(std::move(mesh));
}

bool TrackCollisionMesh::sample(Real x, Real z, Real& height, Vec3& normal) const {
    const physics::MeshRayHit hit = bvh_.raycast(Vec3(x, kRayTop, z), Vec3(0.0, -1.0, 0.0), kRayTop - kRayBottom);
    if (!hit.hit) {
        return false;
    }
    height = hit.point.y;
    normal = hit.normal;
    return true;
}

} // namespace sim::app::drive
