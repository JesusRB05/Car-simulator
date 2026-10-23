// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/GarageScene.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

#include "core/Log.hpp"
#include "session/CarCatalog.hpp"
#include "session/CarVisual.hpp"

namespace sim::app {
namespace {

constexpr float kTyreHalfWidth = 0.12f;

const glm::vec4 kCabinColor{0.10f, 0.12f, 0.16f, 1.0f};
const glm::vec4 kTyreColor{0.06f, 0.06f, 0.07f, 1.0f};
const glm::vec4 kHubColor{0.75f, 0.76f, 0.80f, 1.0f};
const glm::vec4 kHeadlightColor{1.0f, 0.95f, 0.70f, 1.0f};
const glm::vec4 kTaillightColor{0.90f, 0.05f, 0.05f, 1.0f};
const glm::vec4 kFloorColor{0.16f, 0.17f, 0.19f, 1.0f};

// The showroom is a photographic studio rather than an outdoor scene: a light
// neutral backdrop, a strong key from the upper front-left so the bodywork and
// the wheel arches read, and no hard horizon. The sky colour doubles as the
// hemispheric ambient of the scene, so it is the main fill here.
const glm::vec4 kStudioSky{0.30f, 0.31f, 0.34f, 1.0f};
const glm::vec3 kStudioKeyDirection{0.30f, -0.92f, 0.25f}; // the way the light travels
const glm::vec3 kStudioKeyColor{3.0f, 2.95f, 2.85f};       // warm white
constexpr float kStudioFloorSize = 60.0f;

glm::mat4 boxAt(const glm::mat4& frame, const glm::vec3& centre, const glm::vec3& size) {
    return frame * glm::translate(glm::mat4(1.0f), centre) * glm::scale(glm::mat4(1.0f), size);
}

} // namespace

GarageScene::GarageScene(sim::rhi::Device& device, const sim::vfs::VirtualFileSystem& vfs)
    : device_(device),
      vfs_(vfs),
      cube_(sim::render::Mesh::makeCube(device)),
      cylinder_(sim::render::Mesh::makeCylinder(device)),
      floor_(sim::render::Mesh::makeGroundPlane(device, kStudioFloorSize)) {
    camera_.yaw = 0.65f;
    camera_.pitch = 0.22f;
    camera_.fovDegrees = 40.0f;
}

void GarageScene::setCar(const CarSpec& car, const CarSkin* skin) {
    const std::string carRef = car.ref();
    const std::string skinRef = skin != nullptr ? skin->ref : std::string{};
    if (carRef == carRef_ && skinRef == skinRef_) {
        return; // already showing this exact car and livery
    }
    carRef_ = carRef;
    skinRef_ = skinRef;

    carParts_.clear();
    visual_.reset();
    carCenter_ = glm::vec3(0.0f, 0.6f, 0.0f);
    carRadius_ = 3.2f;

    if (!car.modelPath.empty()) {
        std::vector<std::string> warnings;
        visual_ = loadCarVisual(device_, vfs_, car.modelPath, carVisualOptions(skin), &warnings);
        for (const std::string& warning : warnings) {
            core::Log::get("render")->warn("garage: {}", warning);
        }
        if (visual_ && visual_->valid()) {
            const glm::vec3 size = visual_->boundsMax - visual_->boundsMin;
            carCenter_ = 0.5f * (visual_->boundsMin + visual_->boundsMax);
            carRadius_ = std::max(0.5f * glm::length(size), 1.5f);
        } else {
            visual_.reset();
        }
    }

    if (!visual_) {
        buildPrimitives(car);
    }
    frameCamera();
}

void GarageScene::buildPrimitives(const CarSpec& car) {
    const vehicle::VehicleSetup& setup = car.setup;
    const float wheelRadius = static_cast<float>(setup.wheelRadius);
    const float halfTrack = static_cast<float>(setup.halfTrack);
    const float frontAxleZ = static_cast<float>(setup.frontAxleZ);
    const float rearAxleZ = static_cast<float>(setup.rearAxleZ);
    const float restLength = static_cast<float>(setup.restLength);

    const float bodyCenterY = wheelRadius + restLength;
    const glm::mat4 frame = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, bodyCenterY, 0.0f));

    // Body and cabin, with the same proportions the drive scene uses.
    carParts_.push_back({cube_, boxAt(frame, {0.0f, -0.07f, 0.0f}, {1.76f, 0.52f, 4.30f}), car.bodyColor});
    carParts_.push_back({cube_, boxAt(frame, {0.0f, 0.40f, 0.25f}, {1.45f, 0.44f, 2.0f}), kCabinColor});

    for (int side = -1; side <= 1; side += 2) {
        const float s = static_cast<float>(side);
        carParts_.push_back({cube_, boxAt(frame, {s * 0.60f, -0.02f, -2.16f}, {0.32f, 0.12f, 0.04f}), kHeadlightColor});
        carParts_.push_back({cube_, boxAt(frame, {s * 0.62f, 0.00f, 2.16f}, {0.30f, 0.10f, 0.04f}), kTaillightColor});

        for (int axle = 0; axle < 2; ++axle) {
            const float axleZ = axle == 0 ? frontAxleZ : rearAxleZ;
            const glm::vec3 centre(s * halfTrack, wheelRadius, axleZ);
            const glm::mat4 wheelFrame = glm::translate(glm::mat4(1.0f), centre);
            const glm::mat4 tyre = wheelFrame * glm::rotate(glm::mat4(1.0f), 1.5707963f, glm::vec3(0.0f, 0.0f, 1.0f)) *
                                   glm::scale(glm::mat4(1.0f), glm::vec3(wheelRadius, kTyreHalfWidth, wheelRadius));
            carParts_.push_back({cylinder_, tyre, kTyreColor});
            const float outer = s < 0.0f ? -(kTyreHalfWidth + 0.01f) : (kTyreHalfWidth + 0.01f);
            carParts_.push_back({cube_, boxAt(wheelFrame, {outer, 0.0f, 0.0f}, {0.02f, 0.40f, 0.08f}), kHubColor});
        }
    }

    carCenter_ = glm::vec3(0.0f, bodyCenterY + 0.1f, 0.0f);
    carRadius_ = 3.1f;
}

void GarageScene::resetView() {
    camera_.yaw = 0.65f;
    camera_.pitch = 0.22f;
    frameCamera();
}

void GarageScene::frameCamera() {
    // A little tighter than the default fit: the showroom is about looking at
    // the car, and the studio backdrop has nothing else to show.
    camera_.frame(carCenter_ + glm::vec3(0.0f, 0.20f, 0.0f), carRadius_ * 0.78f);
}

void GarageScene::buildScene(sim::render::Scene& scene, float aspect) {
    scene.clear();
    scene.setSkyColor(kStudioSky);
    scene.setDirectionalLight(kStudioKeyDirection, kStudioKeyColor);
    scene.setExposure(1.45f);
    {
        // The floor is a receiver: it must not cast (it would shadow nothing)
        // and it must not stretch the fitted shadow box.
        sim::render::Drawable floor;
        floor.mesh = floor_;
        floor.material.baseColor = kFloorColor;
        floor.castsShadow = false;
        scene.add(std::move(floor));
    }
    if (visual_ && visual_->valid()) {
        // The model's origin is the ground plane, which is the showroom floor.
        const glm::mat4 chassis(1.0f);
        const std::array<float, 4> spin{0.0f, 0.0f, 0.0f, 0.0f};
        addCarVisual(scene, *visual_, chassis, restHubFrames(*visual_, chassis), spin);
    } else {
        for (const sim::render::Drawable& part : carParts_) {
            scene.add(part.mesh, part.transform, part.material);
        }
    }
    camera_.apply(scene.camera(), aspect);
}

} // namespace sim::app
