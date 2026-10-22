// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/TrackVisual.hpp"

#include <cstdint>
#include <iterator>
#include <utility>

#include "drive/TrackModel.hpp"
#include "render/Model.hpp"
#include "rhi/Device.hpp"

namespace sim::app::drive {
namespace {

// Distance (metres from the camera to a tile's centre) at which each level takes
// over. The circuit is 2.8 km long and seen from 0.05 m to 1 km, so the coarse
// level only replaces geometry that is already small on screen; combined with
// the 48 m tiles the far half of the circuit is drawn at a quarter of its
// triangles.
constexpr float kLodDistances[] = {140.0f, 380.0f};

} // namespace

std::shared_ptr<TrackVisual> uploadTrackVisual(rhi::Device& device, const TrackModel& model) {
    if (model.tiles.empty()) {
        return nullptr;
    }
    auto visual = std::make_shared<TrackVisual>();
    visual->boundsMin = model.boundsMin;
    visual->boundsMax = model.boundsMax;

    // One texture per image, one material per glTF material: thousands of kerb
    // and terrain tiles must not each upload their own copy of the atlas.
    const std::vector<render::Material> materials = render::uploadMaterials(device, model.source);

    visual->meshes.reserve(model.tiles.size());
    visual->drawables.reserve(model.tiles.size());
    for (const TrackTile& tile : model.tiles) {
        auto mesh = std::make_shared<render::Mesh>(device, tile.data.vertices, tile.data.indices);
        visual->triangleCount += tile.data.indices.size() / 3;

        render::Drawable drawable;
        drawable.mesh = mesh;
        if (tile.material >= 0 && static_cast<std::size_t>(tile.material) < materials.size()) {
            drawable.material = materials[static_cast<std::size_t>(tile.material)];
        }
        // The lower levels share the vertex buffer; only their index buffers are
        // new, so a chain of three levels costs the geometry once.
        for (const std::vector<std::uint32_t>& lod : tile.lods) {
            if (drawable.lodMeshes.size() >= std::size(kLodDistances)) {
                break;
            }
            const std::size_t level = drawable.lodMeshes.size();
            drawable.lodMeshes.push_back(std::make_shared<render::Mesh>(device, mesh, lod));
            drawable.lodDistances.push_back(kLodDistances[level]);
            visual->lodTriangleCount += lod.size() / 3;
        }
        visual->meshes.push_back(std::move(mesh));
        visual->drawables.push_back(std::move(drawable));
    }
    return visual;
}

void addTrackVisual(render::Scene& scene, const TrackVisual& visual) {
    for (const render::Drawable& drawable : visual.drawables) {
        scene.add(drawable);
    }
}

} // namespace sim::app::drive
