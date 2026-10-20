// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/CarRig.hpp"

#include <cctype>
#include <cstddef>
#include <string>

namespace sim::app {
namespace {

constexpr const char* kCornerNames[4] = {"FL", "FR", "RL", "RR"};

char toUpper(char c) {
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

bool endsWith(std::string_view text, std::string_view suffix) {
    return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// True when this node is the pivot itself (`wheel_FL`), not a child of it.
bool isWheelPivot(std::string_view name) {
    return name.size() == 8 && carCornerFromNodeName(name) >= 0;
}

} // namespace

int carCornerFromNodeName(std::string_view name) {
    // `wheel_` + front/rear + left/right, then either the end of the name or the
    // `_spin`/`_caliper`/`.001` suffix the convention allows.
    if (name.size() < 8 || name.compare(0, 6, "wheel_") != 0) {
        return -1;
    }
    const char axle = toUpper(name[6]);
    const char side = toUpper(name[7]);
    if ((axle != 'F' && axle != 'R') || (side != 'L' && side != 'R')) {
        return -1;
    }
    if (name.size() > 8 && name[8] != '_' && name[8] != '.') {
        return -1;
    }
    return (axle == 'F' ? 0 : 2) + (side == 'L' ? 0 : 1);
}

CarRig buildCarRig(const asset::GltfAsset& asset, std::vector<std::string>* warnings) {
    CarRig rig;
    const auto warn = [&](const std::string& message) {
        if (warnings != nullptr) {
            warnings->push_back("car rig: " + message);
        }
    };

    if (asset.nodes.empty()) {
        warn("the model has no nodes");
        return rig;
    }

    // Chassis parts are placed relative to the first root, so a model exported
    // with a global transform still lines up with the physics.
    if (!asset.roots.empty()) {
        rig.rootNode = asset.roots.front();
        rig.rootWorld = asset.nodes[static_cast<std::size_t>(rig.rootNode)].world;
    } else {
        warn("the model has no root node; chassis parts are placed in model space");
    }
    const glm::mat4 rootInverse = glm::inverse(rig.rootWorld);

    // ---- Wheel pivots -----------------------------------------------------
    for (std::size_t i = 0; i < asset.nodes.size(); ++i) {
        const asset::Node& node = asset.nodes[i];
        if (!isWheelPivot(node.name)) {
            continue;
        }
        const int corner = carCornerFromNodeName(node.name);
        CarRigWheel& wheel = rig.wheels[static_cast<std::size_t>(corner)];
        if (wheel.found) {
            warn("more than one wheel pivot for corner " + std::string(kCornerNames[corner]) + " ('" + node.name +
                 "')");
            continue;
        }
        wheel.found = true;
        wheel.node = static_cast<int>(i);
        const glm::mat4 pivot = rootInverse * node.world;
        wheel.centre = glm::vec3(pivot[3]);
        wheel.rotation = glm::mat3(pivot);
        wheel.radius = node.radius;
        wheel.width = node.width;
        wheel.steered = node.steered;
    }

    // ---- Parts ------------------------------------------------------------
    for (std::size_t i = 0; i < asset.nodes.size(); ++i) {
        const asset::Node& node = asset.nodes[i];
        if (node.mesh < 0) {
            continue;
        }
        CarRigPart part;
        part.node = static_cast<int>(i);
        part.mesh = node.mesh;

        // Walk up to the nearest wheel pivot. A `_spin` node on the way means
        // the geometry rolls with the wheel; anything else inside the pivot
        // (the caliper) steers but does not spin.
        bool spins = false;
        int hub = -1;
        for (int cursor = static_cast<int>(i); cursor >= 0;) {
            const asset::Node& ancestor = asset.nodes[static_cast<std::size_t>(cursor)];
            if (endsWith(ancestor.name, "_spin")) {
                spins = true;
            }
            if (isWheelPivot(ancestor.name)) {
                hub = cursor;
                break;
            }
            cursor = ancestor.parent;
        }

        if (hub < 0) {
            part.kind = CarRigKind::Chassis;
            part.local = rootInverse * node.world;
        } else {
            // A mesh hung straight on the pivot rolls with it; the convention
            // normally puts the rolling geometry under `_spin` instead.
            if (hub == static_cast<int>(i)) {
                spins = true;
            }
            part.corner = carCornerFromNodeName(asset.nodes[static_cast<std::size_t>(hub)].name);
            part.kind = spins ? CarRigKind::WheelSpin : CarRigKind::WheelCaliper;
            part.local = glm::inverse(asset.nodes[static_cast<std::size_t>(hub)].world) * node.world;
            if (!rig.wheels[static_cast<std::size_t>(part.corner)].found) {
                warn("the wheel pivot for corner " + std::string(kCornerNames[part.corner]) +
                     " is not a node of its own");
            }
        }
        rig.parts.push_back(part);
    }

    if (rig.hasWheels()) {
        for (int corner = 0; corner < 4; ++corner) {
            if (!rig.wheels[static_cast<std::size_t>(corner)].found) {
                warn(std::string("the model has no 'wheel_") + kCornerNames[corner] + "' pivot");
            }
        }
    }

    return rig;
}

} // namespace sim::app
