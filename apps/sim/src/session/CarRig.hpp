// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Turns a car glTF's node graph into the pieces the game animates. The car
// convention (`content/base/cars/halvard_gt3/README.md`) names each corner's
// wheel pivot `wheel_FL` and, under it, `wheel_FL_spin` (the part that rolls) and
// `wheel_FL_caliper` (the part that must not). Everything else hangs off the
// chassis.
//
// This is device-free on purpose: the rigging rules are unit-tested against the
// real content model without a Vulkan device, and the GPU upload (CarVisual)
// just maps the result onto meshes.

#include <array>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "asset/Gltf.hpp"

namespace sim::app {

// What a part moves with.
enum class CarRigKind {
    Chassis,      // rigid with the body
    WheelSpin,    // the rolling part of a wheel (tyre, rim, disc)
    WheelCaliper, // the part that steers but does not spin
};

// One mesh-carrying node of the model, placed relative to its rig origin:
// the chassis root for `Chassis`, the corner's wheel pivot for the wheels.
struct CarRigPart {
    int node = -1; // index into GltfAsset::nodes
    int mesh = -1; // index into GltfAsset::meshes
    int material = -1;
    CarRigKind kind = CarRigKind::Chassis;
    int corner = -1; // 0 FL, 1 FR, 2 RL, 3 RR; -1 for the chassis
    glm::mat4 local{1.0f};
};

// A wheel pivot as the model authored it.
struct CarRigWheel {
    bool found = false;
    int node = -1;
    glm::vec3 centre{0.0f};   // model space, pivot translation
    glm::mat3 rotation{1.0f}; // the pivot's authored orientation
    // `extras` written by the exporter, cross-checked against the physics.
    float radius = 0.0f; // 0 when the node has no `radius` extra
    float width = 0.0f;
    bool steered = false;
};

// The whole car: its parts plus the four wheel pivots, in model space.
struct CarRig {
    std::vector<CarRigPart> parts;
    std::array<CarRigWheel, 4> wheels{};
    int rootNode = -1;
    glm::mat4 rootWorld{1.0f};

    bool hasWheels() const { return wheels[0].found || wheels[1].found || wheels[2].found || wheels[3].found; }
};

// Corner index for a wheel node name (`wheel_FL`, `wheel_FL_spin`,
// `wheel_FL_caliper`, case-insensitive suffix), or -1 when the name is not a
// wheel node.
int carCornerFromNodeName(std::string_view name);

// Walks the node graph and builds the rig. `warnings` collects the recoverable
// issues (no root node, a wheel pivot without its `extras`, two pivots for the
// same corner); an asset without nodes yields an empty rig.
CarRig buildCarRig(const asset::GltfAsset& asset, std::vector<std::string>* warnings = nullptr);

} // namespace sim::app
