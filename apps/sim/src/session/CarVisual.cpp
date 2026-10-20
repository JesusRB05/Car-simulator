// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/CarVisual.hpp"

#include <cstddef>
#include <string>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

#include "render/Scene.hpp"
#include "rhi/Device.hpp"
#include "session/CarCatalog.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

// The parts of a car model that roll about the model's +X axis (the car
// convention: `extras.spin_axis = [1, 0, 0]`).
constexpr glm::vec3 kSpinAxis{1.0f, 0.0f, 0.0f};

} // namespace

CarVisualOptions carVisualOptions(const CarSkin* skin) {
    CarVisualOptions options;
    if (skin == nullptr) {
        return options;
    }
    for (const CarSkinTexture& texture : skin->textures) {
        render::MaterialOverride entry;
        // The slot is "<material>.<channel>" or a bare "<channel>".
        const std::size_t dot = texture.slot.find('.');
        if (dot != std::string::npos) {
            entry.material = texture.slot.substr(0, dot);
            entry.slot = texture.slot.substr(dot + 1);
        } else {
            entry.slot = texture.slot;
        }
        entry.file = texture.file;
        options.overrides.push_back(std::move(entry));
    }
    options.hasBodyColor = skin->hasBodyColor;
    options.bodyColor = skin->bodyColor;
    return options;
}

bool CarVisual::hasWheels() const {
    for (const CarRigWheel& wheel : wheels) {
        if (wheel.found) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<CarVisual> loadCarVisual(rhi::Device& device, const vfs::VirtualFileSystem& vfs,
                                         std::string_view modelPath, const CarVisualOptions& options,
                                         std::vector<std::string>* warnings) {
    if (modelPath.empty()) {
        return nullptr;
    }

    std::shared_ptr<render::Model> model = render::loadModel(device, vfs, modelPath, options.overrides, warnings);
    if (model == nullptr) {
        return nullptr;
    }

    auto visual = std::make_shared<CarVisual>();
    const CarRig rig = buildCarRig(model->source, warnings);
    visual->wheels = rig.wheels;
    visual->boundsMin = model->source.boundsMin;
    visual->boundsMax = model->source.boundsMax;

    // Rig entries are per node; the uploaded parts are per primitive, in their
    // own order.
    std::vector<const CarRigPart*> rigByNode(model->source.nodes.size(), nullptr);
    for (const CarRigPart& part : rig.parts) {
        if (part.node >= 0 && static_cast<std::size_t>(part.node) < rigByNode.size()) {
            rigByNode[static_cast<std::size_t>(part.node)] = &part;
        }
    }

    visual->parts.reserve(model->parts.size());
    for (const render::ModelPart& source : model->parts) {
        CarVisualPart part;
        part.mesh = source.mesh;
        part.material = source.material;
        if (source.node >= 0 && static_cast<std::size_t>(source.node) < rigByNode.size()) {
            if (const CarRigPart* entry = rigByNode[static_cast<std::size_t>(source.node)]) {
                part.kind = entry->kind;
                part.corner = entry->corner;
                part.local = entry->local;
            } else {
                part.local = model->nodeWorld(source.node);
            }
        }

        // A flat paint colour from the skin is the fallback for a material that
        // has no base-colour map to replace.
        if (options.hasBodyColor && source.materialIndex >= 0 &&
            static_cast<std::size_t>(source.materialIndex) < model->source.materials.size() &&
            model->source.materials[static_cast<std::size_t>(source.materialIndex)].name == options.paintMaterial &&
            part.material.baseColorTexture == nullptr) {
            part.material.baseColor = options.bodyColor;
        }

        visual->parts.push_back(std::move(part));
    }

    if (!visual->parts.empty() && !visual->hasWheels() && warnings != nullptr) {
        warnings->push_back(std::string(modelPath) + ": no wheel pivots found; the model is drawn rigid");
    }
    return visual;
}

void addCarVisual(render::Scene& scene, const CarVisual& visual, const glm::mat4& chassis,
                  const std::array<glm::mat4, 4>& hubs, const std::array<float, 4>& spin) {
    for (const CarVisualPart& part : visual.parts) {
        if (part.kind == CarRigKind::Chassis || part.corner < 0) {
            scene.add(part.mesh, chassis * part.local, part.material);
            continue;
        }
        const std::size_t corner = static_cast<std::size_t>(part.corner);
        glm::mat4 hub = hubs[corner];
        if (part.kind == CarRigKind::WheelSpin) {
            // Rolling forward (-Z) turns the wheel about -X, so the top of the
            // tyre moves with the car and the contact patch stays put.
            hub = hub * glm::rotate(glm::mat4(1.0f), -spin[corner], kSpinAxis);
        }
        scene.add(part.mesh, hub * part.local, part.material);
    }
}

std::array<glm::mat4, 4> restHubFrames(const CarVisual& visual, const glm::mat4& chassis) {
    std::array<glm::mat4, 4> hubs{glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f)};
    for (std::size_t corner = 0; corner < 4; ++corner) {
        const CarRigWheel& wheel = visual.wheels[corner];
        if (!wheel.found) {
            hubs[corner] = chassis;
            continue;
        }
        hubs[corner] = chassis * glm::translate(glm::mat4(1.0f), wheel.centre) * glm::mat4(wheel.rotation);
    }
    return hubs;
}

} // namespace sim::app
