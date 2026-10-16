// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace sim::app {

// Options parsed from the command line for the offscreen (headless) renderer.
struct HeadlessOptions {
    std::uint32_t frames = 1;
    std::filesystem::path executablePath;
    std::filesystem::path screenshot;
    std::string scene = "default";
    std::uint32_t width = 1280;
    std::uint32_t height = 720;
    bool validation = false;
    // When >= 0, the offscreen target is recreated at frame N to exercise the
    // resize path without a swapchain.
    int resizeFrame = -1;

    // Golden-image regression. With updateGolden the rendered frame is written
    // as the reference; with verifyGolden it is compared against the reference
    // stored at goldenDir/<scene>.png.
    bool updateGolden = false;
    bool verifyGolden = false;
    std::filesystem::path goldenDir;

    // Semantic checks: project each scene probe to screen and require the
    // expected colour to appear at that pixel.
    bool semantic = false;

    // Contact sheets: labelled mosaic PNGs of every physics scene (captured
    // after their settle time) and of every render-test scene.
    std::filesystem::path contactSheet;
    std::filesystem::path renderTestsSheet;

    // Overlays the instanced debug draw (ground grid + world axes) on top of
    // the scene, exercising the debug pipeline in a headless run.
    bool debugDraw = false;

    // Drive-scene capture. `camera` selects the view (chase, cockpit, orbit or
    // side); `time` is how many seconds the autopilot drives before the capture
    // (0 = the car sits on the grid). `yaw`/`pitch` are the orbit angles.
    std::string camera = "chase";
    double time = 0.0;
    float yaw = 0.8f;
    float pitch = 0.35f;

    // Tolerances for the golden comparison.
    double maxDiffPercent = 1.0;
    double minPsnr = 30.0;
};

// Renders `options.frames` frames entirely offscreen (no window, no swapchain),
// writes the last one to a PNG and returns:
//   0  success
//   1  initialisation / Vulkan failure
//   2  the validation layers emitted at least one warning or error
//   3  a golden image mismatch
//   4  a semantic (projected pixel) check failed
int runHeadless(const HeadlessOptions& options);

} // namespace sim::app
