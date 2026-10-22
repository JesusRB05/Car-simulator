// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// The circuit import: the JSON reader, the `track.json` layout and the glTF
// import (drawable tiles + drivable collision with per-triangle friction). These
// are device-free, so the real Sierra Verde content is imported and inspected
// here without a Vulkan device.

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "drive/Json.hpp"
#include "drive/MeshLods.hpp"
#include "drive/TrackLayout.hpp"
#include "drive/TrackModel.hpp"
#include "render/Mesh.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace {

namespace fs = std::filesystem;
namespace physics = sim::physics;

// A height field on a regular grid: `cells` x `cells` quads over `size` metres,
// with a smooth bump so the simplifier has curvature to work with. The border
// vertices are the ones with a zero coordinate: they are what a neighbouring
// tile shares.
sim::render::MeshData gridMesh(int cells, float size) {
    sim::render::MeshData mesh;
    const int stride = cells + 1;
    mesh.vertices.reserve(static_cast<std::size_t>(stride) * static_cast<std::size_t>(stride));
    for (int z = 0; z <= cells; ++z) {
        for (int x = 0; x <= cells; ++x) {
            sim::render::Vertex vertex;
            const float fx = static_cast<float>(x) / static_cast<float>(cells);
            const float fz = static_cast<float>(z) / static_cast<float>(cells);
            vertex.position = glm::vec3(fx * size, 0.4f * std::sin(fx * 9.0f) * std::cos(fz * 7.0f), fz * size);
            mesh.vertices.push_back(vertex);
        }
    }
    for (int z = 0; z < cells; ++z) {
        for (int x = 0; x < cells; ++x) {
            const std::uint32_t a = static_cast<std::uint32_t>(z * stride + x);
            const std::uint32_t b = a + 1;
            const std::uint32_t c = a + static_cast<std::uint32_t>(stride);
            const std::uint32_t d = c + 1;
            mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
        }
    }
    return mesh;
}

std::set<std::pair<std::uint32_t, std::uint32_t>> edgesOf(const std::vector<std::uint32_t>& indices) {
    std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
    for (std::size_t triangle = 0; triangle + 2 < indices.size(); triangle += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            const std::uint32_t a = indices[triangle + static_cast<std::size_t>(corner)];
            const std::uint32_t b = indices[triangle + static_cast<std::size_t>((corner + 1) % 3)];
            edges.insert({std::min(a, b), std::max(a, b)});
        }
    }
    return edges;
}

// The edges of an open surface that belong to a single triangle: the ones a
// neighbouring tile shares.
std::set<std::pair<std::uint32_t, std::uint32_t>> boundaryEdgesOf(const std::vector<std::uint32_t>& indices) {
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> counts;
    for (std::size_t triangle = 0; triangle + 2 < indices.size(); triangle += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            const std::uint32_t a = indices[triangle + static_cast<std::size_t>(corner)];
            const std::uint32_t b = indices[triangle + static_cast<std::size_t>((corner + 1) % 3)];
            ++counts[{std::min(a, b), std::max(a, b)}];
        }
    }
    std::set<std::pair<std::uint32_t, std::uint32_t>> boundary;
    for (const auto& [edge, count] : counts) {
        if (count == 1) {
            boundary.insert(edge);
        }
    }
    return boundary;
}

fs::path sourceDir() {
    return fs::path(SIM_SOURCE_DIR);
}

constexpr const char* kSierraModel = "tracks/sierra_verde/model/track.glb";
constexpr const char* kSierraLayout = "tracks/sierra_verde/track.json";

// Mounting the official content is not free (PhysFS sets up its search path), so
// the cases that need it share one mount.
sim::vfs::VirtualFileSystem& contentVfs() {
    static sim::vfs::VirtualFileSystem vfs;
    static const bool mounted = vfs.mount(sourceDir() / "content" / "base");
    (void)mounted;
    return vfs;
}

TEST_CASE("lods: a simplified tile keeps the border a neighbour shares") {
    // A regular grid stands in for one terrain tile: it has a real border and a
    // little curvature, so the simplifier can work but must not touch the rim.
    constexpr int kCells = 64;
    const sim::render::MeshData grid = gridMesh(kCells, 48.0f);

    sim::app::drive::LodOptions options;
    options.ratios = {0.5f};
    options.minTriangles = 64;
    const std::vector<std::vector<std::uint32_t>> levels = sim::app::drive::buildMeshLods(grid, options);
    REQUIRE(levels.size() == 1);
    const std::vector<std::uint32_t>& lod = levels.front();
    CHECK(lod.size() < grid.indices.size());
    CHECK(lod.size() > grid.indices.size() / 4);

    // Every vertex on the rim survives, index for index: the vertices are shared
    // with the neighbouring tiles, which is what keeps the seam closed.
    const std::set<std::uint32_t> lodVertices(lod.begin(), lod.end());
    const int stride = kCells + 1;
    std::size_t lostVertices = 0;
    for (int z = 0; z <= kCells; ++z) {
        for (int x = 0; x <= kCells; ++x) {
            if (x != 0 && x != kCells && z != 0 && z != kCells) {
                continue;
            }
            if (lodVertices.count(static_cast<std::uint32_t>(z * stride + x)) == 0) {
                ++lostVertices;
            }
        }
    }
    CHECK(lostVertices == 0);

    // ... and so does every border edge.
    const auto border = boundaryEdgesOf(grid.indices);
    const auto lodEdges = edgesOf(lod);
    std::size_t lostEdges = 0;
    for (const auto& edge : border) {
        if (lodEdges.count(edge) == 0) {
            ++lostEdges;
        }
    }
    CHECK(lostEdges == 0);
    MESSAGE("grid " << grid.indices.size() / 3 << " -> " << lod.size() / 3 << " triangles, " << border.size()
                    << " border edges");
}

TEST_CASE("lods: a mesh below the threshold gets no chain") {
    const sim::render::MeshData grid = gridMesh(4, 8.0f);
    sim::app::drive::LodOptions options;
    options.minTriangles = 64; // the grid has 32 triangles
    CHECK(sim::app::drive::buildMeshLods(grid, options).empty());
}

TEST_CASE("model: the import builds a LOD chain that cannot crack the tiles") {
    sim::app::drive::TrackModel model;
    std::string error;
    REQUIRE_MESSAGE(sim::app::drive::loadTrackModel(contentVfs(), kSierraModel, {}, model, error), error);

    // A tile's real border is geometric: glTF duplicates a vertex per texture
    // seam, so the index edges alone would report seams as borders. Grouping by
    // position first, an edge with a single triangle is on the border; those
    // positions are what a neighbouring tile shares.
    using PositionKey = std::tuple<std::uint32_t, std::uint32_t, std::uint32_t>;
    const auto positionKey = [](const glm::vec3& position) {
        const auto bits = [](float value) {
            std::uint32_t raw = 0;
            std::memcpy(&raw, &value, sizeof(raw));
            return raw == 0x80000000u ? 0u : raw;
        };
        return std::make_tuple(bits(position.x), bits(position.y), bits(position.z));
    };
    const auto borderOf = [&](const std::vector<sim::render::Vertex>& vertices,
                              const std::vector<std::uint32_t>& indices) {
        std::map<std::pair<PositionKey, PositionKey>, int> counts;
        for (std::size_t triangle = 0; triangle + 2 < indices.size(); triangle += 3) {
            for (int corner = 0; corner < 3; ++corner) {
                const PositionKey a =
                    positionKey(vertices[indices[triangle + static_cast<std::size_t>(corner)]].position);
                const PositionKey b =
                    positionKey(vertices[indices[triangle + static_cast<std::size_t>((corner + 1) % 3)]].position);
                ++counts[{std::min(a, b), std::max(a, b)}];
            }
        }
        std::set<PositionKey> borderPositions;
        std::set<std::pair<PositionKey, PositionKey>> borderEdges;
        for (const auto& [edge, count] : counts) {
            if (count != 1) {
                continue;
            }
            borderEdges.insert(edge);
            borderPositions.insert(edge.first);
            borderPositions.insert(edge.second);
        }
        return std::make_pair(std::move(borderPositions), std::move(borderEdges));
    };
    const auto reachableFrom = [&](const std::vector<sim::render::Vertex>& vertices,
                                   const std::vector<std::uint32_t>& indices) {
        std::set<PositionKey> positions;
        std::set<std::pair<PositionKey, PositionKey>> edges;
        for (const std::uint32_t index : indices) {
            positions.insert(positionKey(vertices[index].position));
        }
        for (std::size_t triangle = 0; triangle + 2 < indices.size(); triangle += 3) {
            for (int corner = 0; corner < 3; ++corner) {
                const PositionKey a =
                    positionKey(vertices[indices[triangle + static_cast<std::size_t>(corner)]].position);
                const PositionKey b =
                    positionKey(vertices[indices[triangle + static_cast<std::size_t>((corner + 1) % 3)]].position);
                edges.insert({std::min(a, b), std::max(a, b)});
            }
        }
        return std::make_pair(std::move(positions), std::move(edges));
    };

    std::size_t tilesWithLods = 0;
    std::size_t baseTriangles = 0;
    std::size_t lodTriangles = 0;
    std::size_t lostBorderVertices = 0;
    std::size_t lostBorderEdges = 0;
    for (const sim::app::drive::TrackTile& tile : model.tiles) {
        baseTriangles += tile.data.indices.size() / 3;
        if (tile.lods.empty()) {
            continue;
        }
        ++tilesWithLods;
        CHECK(tile.lods.size() <= 2);
        const auto [border, borderEdges] = borderOf(tile.data.vertices, tile.data.indices);
        std::size_t previous = tile.data.indices.size();
        for (const std::vector<std::uint32_t>& lod : tile.lods) {
            REQUIRE(lod.size() % 3 == 0);
            CHECK(lod.size() >= 3);
            CHECK(lod.size() < previous);
            previous = lod.size();
            std::size_t outOfRange = 0;
            for (const std::uint32_t index : lod) {
                outOfRange += index >= tile.data.vertices.size() ? 1 : 0;
            }
            CHECK(outOfRange == 0);
            // The simplified tile still covers every position and every edge of
            // the shared border, so it lines up with a neighbour drawn at full
            // detail.
            const auto [positions, edges] = reachableFrom(tile.data.vertices, lod);
            for (const PositionKey& position : border) {
                if (positions.count(position) == 0) {
                    ++lostBorderVertices;
                }
            }
            for (const auto& edge : borderEdges) {
                if (edges.count(edge) == 0) {
                    ++lostBorderEdges;
                }
            }
            lodTriangles += lod.size() / 3;
        }
    }

    // The big surfaces (terrain, gravel, grass) are the ones worth a chain, and
    // the chain as a whole is much cheaper than the base level.
    CHECK(tilesWithLods > 20);
    CHECK(baseTriangles > 300000);
    CHECK(lodTriangles < baseTriangles);
    MESSAGE("circuit LOD chain: " << tilesWithLods << " tile(s), " << baseTriangles << " base triangles, "
                                  << lodTriangles << " more in the chain, lost border " << lostBorderVertices
                                  << " vertex/vertices and " << lostBorderEdges << " edge(s)");
    // A lost border vertex or edge would open a crack between two tiles drawn at
    // different levels.
    CHECK(lostBorderVertices == 0);
    CHECK(lostBorderEdges == 0);
}

TEST_CASE("model: the imported circuit pins its start pose and grid") {
    // The layout, not the mesh, says where the line is; the drive scene uses it
    // to spawn the car. This pins the authored value so a content edit cannot
    // silently move the start.
    std::string error;
    const auto layout = sim::app::drive::loadTrackLayout(contentVfs(), kSierraLayout, error);
    REQUIRE_MESSAGE(layout.has_value(), error);
    REQUIRE(layout->hasStart);
    CHECK(layout->start.x == doctest::Approx(0.0));
    CHECK(layout->start.z == doctest::Approx(0.0));
    CHECK(layout->startHeadingRadians == doctest::Approx(0.0));

    // On the grid, the wheel query answers with the road, not with the grass
    // under it, at every slot.
    sim::app::drive::TrackModel model;
    REQUIRE(sim::app::drive::loadTrackModel(contentVfs(), kSierraModel, {}, model, error));
    const sim::app::drive::TrackSurfaceQuery surface(model.collision, model.triangleFriction);
    for (const sim::app::drive::TrackLayout::GridSlot& slot : layout->grid) {
        physics::Real height = 0.0;
        physics::Vec3 normal(0.0, 1.0, 0.0);
        physics::Real friction = 0.0;
        REQUIRE_MESSAGE(surface.sample(slot.x, slot.z, height, normal, friction),
                        "no ground under grid slot " << slot.x << ", " << slot.z);
        CHECK(std::abs(height) < 0.5);
        CHECK(normal.y > 0.95);
        CHECK(friction >= 0.85);
    }
}

} // namespace

TEST_CASE("json: a document parses into numbers, strings, arrays and objects") {
    std::string error;
    const auto document = sim::app::json::parse(R"({
        "name": "Sierra \u00e9 Verde",
        "length": 2784.27,
        "exponent": -1.5e3,
        "ok": true,
        "nothing": null,
        "points": [[0, 0, 0], [1.5, -2, 3]]
    })",
                                                error);
    REQUIRE(document.has_value());
    CHECK(document->isObject());
    const auto* name = document->find("name");
    REQUIRE(name != nullptr);
    CHECK(name->text == "Sierra \xC3\xA9 Verde");
    const auto* length = document->find("length");
    REQUIRE(length != nullptr);
    CHECK(length->number == doctest::Approx(2784.27));
    CHECK(document->find("exponent")->number == doctest::Approx(-1500.0));
    CHECK(document->find("ok")->boolean);
    CHECK(document->find("nothing")->isNull());
    const auto* points = document->find("points");
    REQUIRE(points != nullptr);
    REQUIRE(points->isArray());
    REQUIRE(points->items.size() == 2);
    CHECK(points->items[1].items[1].number == doctest::Approx(-2.0));
}

TEST_CASE("json: malformed documents are rejected with a reason") {
    const char* bad[] = {
        "", "{", "{\"a\": }", "[1, 2,]", "{} trailing", "{\"a\": 1, \"a\": 2}", "01", "{\"a\": 1e}", "\"unterminated",
    };
    for (const char* text : bad) {
        std::string error;
        const auto document = sim::app::json::parse(text, error);
        CHECK_MESSAGE(!document.has_value(), "accepted: " << text);
        CHECK(!error.empty());
    }
}

TEST_CASE("layout: the Sierra Verde track.json describes the real circuit") {
    std::string error;
    const auto layout = sim::app::drive::loadTrackLayout(contentVfs(), kSierraLayout, error);
    REQUIRE_MESSAGE(layout.has_value(), error);

    // 2.78 km authored every ~4 m.
    CHECK(layout->centreline.size() == 696);
    CHECK(layout->lengthMeters == doctest::Approx(2784.3).epsilon(0.001));
    CHECK(layout->halfWidthMeters == doctest::Approx(6.5));

    // The line is at the origin and the cars face -Z there (heading 0).
    REQUIRE(layout->hasStart);
    CHECK(layout->start.x == doctest::Approx(0.0));
    CHECK(layout->start.z == doctest::Approx(0.0));
    CHECK(layout->startHeadingRadians == doctest::Approx(0.0));

    // Twelve grid slots, all behind the line (+Z) and facing the same way.
    REQUIRE(layout->grid.size() == 12);
    for (const auto& slot : layout->grid) {
        CHECK(slot.z >= 0.0);
        CHECK(std::abs(slot.headingRadians) < 0.05);
    }
    // The first slot is the closest to the line.
    for (const auto& slot : layout->grid) {
        CHECK(slot.z >= layout->grid.front().z - 1.0e-9);
    }

    // Three sectors: the split distances are strictly increasing and end at the
    // lap length.
    REQUIRE(layout->sectorDistances.size() >= 3);
    CHECK(layout->sectorDistances.front() == doctest::Approx(0.0));
    for (std::size_t i = 1; i < layout->sectorDistances.size(); ++i) {
        CHECK(layout->sectorDistances[i] > layout->sectorDistances[i - 1]);
    }
    CHECK(layout->sectorDistances.back() <= layout->lengthMeters + 1.0);

    // The pit lane is a 60 km/h polyline on the +X side.
    CHECK(layout->pitLane.speedLimitKmh == doctest::Approx(60.0));
    CHECK(layout->pitLane.polyline.size() >= 3);
    for (const auto& point : layout->pitLane.polyline) {
        CHECK(point.x > layout->halfWidthMeters);
    }

    // The authored elevation is real: the lap climbs and descends.
    double minY = layout->centreline.front().y;
    double maxY = minY;
    for (const auto& point : layout->centreline) {
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }
    CHECK(maxY - minY > 5.0);
}

TEST_CASE("layout: a malformed track.json is rejected") {
    std::string error;
    CHECK(!sim::app::drive::parseTrackLayout("{}", "track.json", error).has_value());
    CHECK(!error.empty());
    CHECK(!sim::app::drive::parseTrackLayout(R"({"centerline": [[0,0,0],[1,0,0]]})", "track.json", error).has_value());
    CHECK(!sim::app::drive::parseTrackLayout(R"({"centerline": [[0,0,0],[1,0,0],[0,0,-1]],
                                                  "half_width_m": 0})",
                                             "track.json", error)
               .has_value());
    CHECK(!sim::app::drive::parseTrackLayout(R"({"centerline": [[0,0,0],[1,0,0],[0,0,-1]],
                                                  "sectors_m": [0, 5, 3]})",
                                             "track.json", error)
               .has_value());
}

TEST_CASE("model: the Sierra Verde glTF imports tiles and a drivable surface") {
    sim::app::drive::TrackModel model;
    std::string error;
    REQUIRE_MESSAGE(sim::app::drive::loadTrackModel(contentVfs(), kSierraModel, {}, model, error), error);
    for (const std::string& warning : model.warnings) {
        MESSAGE("import warning: " << warning);
    }

    // Every node with a mesh becomes at least one drawable, and the large ones
    // (terrain, barriers) are split into tiles.
    CHECK(model.tiles.size() > 100);
    CHECK(model.tiles.size() < 2000);
    std::size_t drawn = 0;
    for (const auto& tile : model.tiles) {
        CHECK(!tile.data.indices.empty());
        CHECK(tile.data.indices.size() % 3 == 0);
        CHECK(tile.material >= 0);
        CHECK(tile.material < static_cast<int>(model.source.materials.size()));
        drawn += tile.data.indices.size() / 3;
    }
    CHECK(drawn > 300000);

    // The collision surface is exactly the ground: the drivable materials are
    // asphalt (1.00), asphalt run-off (0.95), kerb (0.90), concrete (0.90),
    // paint (0.85), grass (0.55) and gravel (0.45). Nothing above a wall or
    // under a grandstand may enter it, or a downward ray would land on a roof.
    REQUIRE(!model.collision.indices.empty());
    REQUIRE(model.collision.indices.size() % 3 == 0);
    REQUIRE(model.triangleFriction.size() == model.collision.indices.size() / 3);
    const std::set<float> allowed = {1.0f, 0.95f, 0.9f, 0.85f, 0.55f, 0.45f};
    std::set<float> seen;
    for (const float friction : model.triangleFriction) {
        CHECK(allowed.count(friction) == 1);
        seen.insert(friction);
    }
    // The lap touches at least asphalt and grass, and the authored gravel traps
    // and kerbs are in the mesh too.
    CHECK(seen.count(1.0f) == 1);
    CHECK(seen.count(0.55f) == 1);
    CHECK(seen.count(0.45f) == 1);
    CHECK(seen.count(0.9f) == 1);

    // The surface covers the whole circuit footprint.
    CHECK(model.boundsMax.x - model.boundsMin.x > 500.0f);
    CHECK(model.boundsMax.z - model.boundsMin.z > 500.0f);
    CHECK(model.boundsMax.y > model.boundsMin.y + 5.0f);
}

TEST_CASE("model: the drivable surface answers the wheel query with its friction") {
    sim::app::drive::TrackModel model;
    std::string error;
    REQUIRE(sim::app::drive::loadTrackModel(contentVfs(), kSierraModel, {}, model, error));

    sim::app::drive::TrackSurfaceQuery surface(model.collision, model.triangleFriction);
    CHECK(surface.triangleCount() == model.collision.indices.size() / 3);

    // The road is generated as two halves that meet on the centreline. A ray on
    // that seam sits exactly on a BVH face and on a shared triangle edge, which
    // used to be pruned: the start line and the racing line would then fall
    // through to the grass 0.43 m below.
    for (const double x : {0.0, 1.0e-9, -1.0e-6, 0.25, -0.25}) {
        physics::Real seamHeight = 0.0;
        physics::Vec3 seamNormal(0.0, 1.0, 0.0);
        physics::Real seamFriction = 0.0;
        REQUIRE_MESSAGE(surface.sample(x, -60.0, seamHeight, seamNormal, seamFriction), "no hit at x=" << x);
        CHECK(std::abs(seamHeight) < 0.1);
        CHECK(seamFriction == doctest::Approx(1.0).epsilon(0.01));
    }

    // The start line is at the origin and the road is effectively flat there.
    physics::Real height = 0.0;
    physics::Vec3 normal(0.0, 1.0, 0.0);
    physics::Real friction = 0.0;
    REQUIRE(surface.sample(0.0, 0.0, height, normal, friction));
    CHECK(std::abs(height) < 0.2);
    CHECK(normal.y > 0.9);
    // The line itself is painted; a metre to the side is plain asphalt.
    CHECK(friction >= 0.85);

    // The main straight: asphalt, level, drivable.
    REQUIRE(surface.sample(0.0, -100.0, height, normal, friction));
    CHECK(friction == doctest::Approx(1.0).epsilon(0.01));
    CHECK(normal.y > 0.95);

    // A grid of samples across the circuit must find every surface, including
    // the grass beside the track and the gravel traps.
    std::set<float> frictions;
    int hits = 0;
    for (double x = model.boundsMin.x; x < model.boundsMax.x; x += 17.0) {
        for (double z = model.boundsMin.z; z < model.boundsMax.z; z += 17.0) {
            if (surface.sample(x, z, height, normal, friction)) {
                frictions.insert(static_cast<float>(friction));
                ++hits;
            }
        }
    }
    CHECK(hits > 500);
    CHECK(frictions.count(1.0f) == 1);  // asphalt
    CHECK(frictions.count(0.55f) == 1); // grass
    CHECK(frictions.count(0.45f) == 1); // gravel

    // Off the mapped area the query reports no hit instead of inventing ground.
    CHECK(!surface.sample(5000.0, 5000.0, height, normal, friction));
}
