// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/TrackVisual.hpp"

#include <utility>

#include "drive/TrackModel.hpp"
#include "render/Model.hpp"
#include "rhi/Device.hpp"

namespace sim::app::drive {

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
        visual->meshes.push_back(std::move(mesh));
        visual->drawables.push_back(std::move(drawable));
    }
    return visual;
}

void addTrackVisual(render::Scene& scene, const TrackVisual& visual) {
    for (const render::Drawable& drawable : visual.drawables) {
        scene.add(drawable.mesh, drawable.transform, drawable.material);
    }
}

} // namespace sim::app::drive
