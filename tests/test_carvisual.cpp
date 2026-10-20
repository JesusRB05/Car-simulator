// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "asset/Gltf.hpp"
#include "modding/Composition.hpp"
#include "session/CarCatalog.hpp"
#include "session/CarRig.hpp"
#include "session/ContentOwner.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace {

namespace fs = std::filesystem;

fs::path sourceDir() {
    return fs::path(SIM_SOURCE_DIR);
}

sim::app::ContentOwner ownerFor(const sim::vfs::VirtualFileSystem& vfs) {
    return [&vfs](std::string_view path) -> std::string {
        const std::string real = vfs.realDirectoryOf(path);
        if (real.find("example_mod") != std::string::npos) {
            return "example.hatchback";
        }
        return "base.core";
    };
}

// Loads the car's model without decoding its (large) images: the rig only reads
// the node graph and the extras.
sim::asset::GltfLoadResult loadCarModel(const sim::vfs::VirtualFileSystem& vfs, const std::string& path) {
    sim::asset::GltfLoadOptions options;
    options.decodeImages = false;
    options.generateTangents = false;
    return sim::asset::loadGltf(vfs, path, options);
}

} // namespace

TEST_CASE("car rig: node names identify the corner and the rolling part") {
    using sim::app::carCornerFromNodeName;
    CHECK(carCornerFromNodeName("wheel_FL") == 0);
    CHECK(carCornerFromNodeName("wheel_FR") == 1);
    CHECK(carCornerFromNodeName("wheel_RL") == 2);
    CHECK(carCornerFromNodeName("wheel_RR") == 3);
    CHECK(carCornerFromNodeName("wheel_FL_spin") == 0);
    CHECK(carCornerFromNodeName("wheel_RR_caliper") == 3);
    // Blender's duplicate suffix is allowed; anything else is not a wheel.
    CHECK(carCornerFromNodeName("wheel_FL.001") == 0);
    CHECK(carCornerFromNodeName("wheel_fl") == 0);
    CHECK(carCornerFromNodeName("wheel_XL") == -1);
    CHECK(carCornerFromNodeName("wheel_F") == -1);
    CHECK(carCornerFromNodeName("wheel_FLsomething") == -1);
    CHECK(carCornerFromNodeName("body") == -1);
    CHECK(carCornerFromNodeName("") == -1);
}

TEST_CASE("car rig: the Halvard GT3 pivots, radii and rolling parts come from the model") {
    sim::vfs::VirtualFileSystem vfs;
    REQUIRE(vfs.mount(sourceDir() / "content" / "base"));

    const sim::asset::GltfLoadResult loaded = loadCarModel(vfs, "cars/halvard_gt3/model/car.glb");
    for (const std::string& error : loaded.errors) {
        MESSAGE("car.glb error: " << error);
    }
    REQUIRE(loaded.ok());

    std::vector<std::string> warnings;
    const sim::app::CarRig rig = sim::app::buildCarRig(*loaded.asset, &warnings);
    for (const std::string& warning : warnings) {
        MESSAGE("rig warning: " << warning);
    }
    CHECK(warnings.empty());
    REQUIRE(rig.hasWheels());

    // The pivots are the model's `wheel_XX` nodes: the physics wheel mounts and
    // the extras both come from here.
    const double expectedX[4] = {-0.835, 0.835, -0.835, 0.835};
    const double expectedY[4] = {0.340, 0.340, 0.355, 0.355};
    const double expectedZ[4] = {-1.38, -1.38, 1.32, 1.32};
    const double expectedRadius[4] = {0.340, 0.340, 0.355, 0.355};
    for (int corner = 0; corner < 4; ++corner) {
        const sim::app::CarRigWheel& wheel = rig.wheels[static_cast<std::size_t>(corner)];
        CAPTURE(corner);
        REQUIRE(wheel.found);
        CHECK(wheel.centre.x == doctest::Approx(expectedX[corner]).epsilon(1e-4));
        CHECK(wheel.centre.y == doctest::Approx(expectedY[corner]).epsilon(1e-4));
        CHECK(wheel.centre.z == doctest::Approx(expectedZ[corner]).epsilon(1e-4));
        CHECK(wheel.radius == doctest::Approx(expectedRadius[corner]).epsilon(1e-4));
        // Only the front axle steers.
        CHECK(wheel.steered == (corner < 2));
    }

    // Every mesh hangs off the chassis or a corner, and the tyre/rim/disc roll
    // while the caliper steers without spinning.
    int chassisParts = 0;
    int spinParts[4] = {0, 0, 0, 0};
    int caliperParts[4] = {0, 0, 0, 0};
    for (const sim::app::CarRigPart& part : rig.parts) {
        if (part.kind == sim::app::CarRigKind::Chassis) {
            ++chassisParts;
            CHECK(part.corner == -1);
            continue;
        }
        REQUIRE(part.corner >= 0);
        REQUIRE(part.corner < 4);
        if (part.kind == sim::app::CarRigKind::WheelSpin) {
            ++spinParts[part.corner];
        } else {
            ++caliperParts[part.corner];
        }
    }
    CHECK(chassisParts == 4); // body, aero, details, interior
    for (int corner = 0; corner < 4; ++corner) {
        CAPTURE(corner);
        CHECK(spinParts[corner] == 1);    // the `_spin` node (tyre, rim, disc, nut)
        CHECK(caliperParts[corner] == 1); // the caliper itself does not spin
    }
    CHECK(rig.parts.size() == 12);
}
