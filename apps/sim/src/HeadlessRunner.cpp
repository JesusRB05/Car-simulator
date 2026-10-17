// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "HeadlessRunner.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <stb_easy_font.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include "PhysicsScenes.hpp"
#include "Scenes.hpp"
#include "core/Log.hpp"
#include "core/Version.hpp"
#include "drive/DriveScene.hpp"
#include "modding/Package.hpp"
#include "physics/PhysicsWorld.hpp"
#include "render/DebugDraw.hpp"
#include "render/Mesh.hpp"
#include "render/Renderer.hpp"
#include "render/Scene.hpp"
#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"
#include "rhi/Resources.hpp"
#include "rhi/Types.hpp"
#include "session/CarCatalog.hpp"
#include "session/ContentOwner.hpp"
#include "session/TrackCatalog.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

constexpr rhi::Format kColorFormat = rhi::Format::R8G8B8A8Unorm;

std::filesystem::path searchForRoot(const std::filesystem::path& exeDir) {
    std::vector<std::filesystem::path> starts{std::filesystem::current_path(), exeDir};
    for (const std::filesystem::path& start : starts) {
        std::filesystem::path dir = start;
        for (int depth = 0; depth < 8 && !dir.empty(); ++depth) {
            if (std::filesystem::exists(dir / "content" / "base")) {
                return dir;
            }
            const std::filesystem::path parent = dir.parent_path();
            if (parent == dir) {
                break;
            }
            dir = parent;
        }
    }
    return exeDir;
}

// Maps a `--camera` value to a drive-scene camera. Unknown names fall back to
// the chase camera so a typo never aborts a capture.
drive::DriveCamera driveCameraFromName(const std::string& name) {
    if (name == "cockpit") {
        return drive::DriveCamera::Cockpit;
    }
    if (name == "orbit") {
        return drive::DriveCamera::Orbit;
    }
    if (name == "side") {
        return drive::DriveCamera::Side;
    }
    return drive::DriveCamera::Chase;
}

void submitAndWait(rhi::Device& device, rhi::CommandList& commandList) {
    VkFence fence = VK_NULL_HANDLE;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (vkCreateFence(device.handle(), &fenceInfo, nullptr, &fence) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a fence");
    }

    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    const VkCommandBuffer handle = commandList.handle();
    submit.pCommandBuffers = &handle;

    vkQueueSubmit(device.graphicsQueue(), 1, &submit, fence);
    vkWaitForFences(device.handle(), 1, &fence, VK_TRUE, UINT64_MAX);
    vkDestroyFence(device.handle(), fence, nullptr);
}

struct Offscreen {
    rhi::Extent2D extent{};
    std::unique_ptr<rhi::Image> color;
    std::unique_ptr<rhi::Buffer> readback;
};

void createTargets(rhi::Device& device, Offscreen& target, rhi::Extent2D extent) {
    target.extent = extent;
    target.color = std::make_unique<rhi::Image>(device, extent, kColorFormat,
                                                rhi::ImageUsage::ColorAttachment | rhi::ImageUsage::TransferSrc);
    const std::size_t size = static_cast<std::size_t>(extent.width) * extent.height * 4;
    target.readback =
        std::make_unique<rhi::Buffer>(device, size, rhi::BufferUsage::TransferDst, rhi::MemoryUsage::GpuToCpu);
}

// Renders `frames` frames into the offscreen target and returns the last frame's
// RGBA pixels (top-left origin, matching the PNG writer). When `debug` is given
// the grid/axes overlay is drawn; `decorate` (used with physics scenes) appends
// scene-specific lines on top of that overlay.
std::vector<std::uint8_t> renderFrames(rhi::Device& device, render::Renderer& renderer, Offscreen& target,
                                       render::Scene& scene, std::uint32_t frames, int resizeFrame, bool logResize,
                                       render::DebugDraw* debug = nullptr,
                                       const std::function<void(render::DebugDraw&)>& decorate = {}) {
    rhi::CommandList commandList(device);
    for (std::uint32_t frame = 0; frame < frames; ++frame) {
        if (resizeFrame >= 0 && frame == static_cast<std::uint32_t>(resizeFrame)) {
            device.waitIdle();
            target.color.reset();
            target.readback.reset();
            const rhi::Extent2D smaller{std::max(target.extent.width / 2, 1u), std::max(target.extent.height / 2, 1u)};
            createTargets(device, target, smaller);
            renderer.ensureTargets(target.extent, kColorFormat);
            if (logResize) {
                std::printf("[headless] resized offscreen target to %ux%u at frame %u\n", target.extent.width,
                            target.extent.height, frame);
            }
        }

        const bool last = frame + 1 == frames;

        commandList.begin();
        commandList.transitionImage(target.color->handle(), rhi::ImageLayout::Undefined,
                                    rhi::ImageLayout::ColorAttachment);

        rhi::ColorAttachment attachment;
        attachment.image = reinterpret_cast<std::uint64_t>(target.color->view());
        attachment.layout = rhi::ImageLayout::ColorAttachment;
        attachment.loadOp = rhi::LoadOp::Clear;
        const glm::vec4& sky = scene.skyColor();
        std::memcpy(attachment.clearColor, &sky.r, sizeof(attachment.clearColor));

        renderer.updateFrameUniforms(0, scene.camera(), scene.lightDirection(), scene.lightColor());
        renderer.renderScene(commandList, scene, attachment, 0);

        if (debug != nullptr) {
            rhi::ColorAttachment overlay = attachment;
            overlay.loadOp = rhi::LoadOp::Load;
            debug->clear();
            debug->addGrid(20.0f, 1.0f, {0.55f, 0.58f, 0.62f, 1.0f}, {0.75f, 0.30f, 0.20f, 1.0f});
            debug->addAxes({0.0f, 0.01f, 0.0f}, 3.0f);
            if (decorate) {
                decorate(*debug);
            }
            debug->render(commandList, scene.camera(), overlay);
        }

        if (last) {
            commandList.transitionImage(target.color->handle(), rhi::ImageLayout::ColorAttachment,
                                        rhi::ImageLayout::TransferSrc);
            commandList.copyImageToBuffer(target.color->handle(), target.readback->handle(), target.extent);
        }
        commandList.end();
        submitAndWait(device, commandList);
    }

    const std::size_t size = static_cast<std::size_t>(target.extent.width) * target.extent.height * 4;
    std::vector<std::uint8_t> pixels(size);
    if (const void* mapped = target.readback->mappedData()) {
        std::memcpy(pixels.data(), mapped, size);
    }
    return pixels;
}

// ---- PNG helpers -----------------------------------------------------------

bool writePng(const std::filesystem::path& path, const std::vector<std::uint8_t>& pixels, rhi::Extent2D extent) {
    if (!path.parent_path().empty()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
    }
    const int stride = static_cast<int>(extent.width) * 4;
    const int written = stbi_write_png(path.string().c_str(), static_cast<int>(extent.width),
                                       static_cast<int>(extent.height), 4, pixels.data(), stride);
    return written != 0;
}

// Compares `pixels` against the reference PNG. Returns true when the image is
// within tolerance; fills the measured metrics for reporting.
bool compareGolden(const std::filesystem::path& reference, const std::vector<std::uint8_t>& pixels,
                   rhi::Extent2D extent, double maxDiffPercent, double minPsnr, double& diffPercent, double& psnr) {
    int rw = 0;
    int rh = 0;
    int channels = 0;
    unsigned char* ref = stbi_load(reference.string().c_str(), &rw, &rh, &channels, 4);
    if (ref == nullptr) {
        std::fprintf(stderr, "[headless] cannot read golden reference %s\n", reference.string().c_str());
        return false;
    }

    const bool sameSize = rw == static_cast<int>(extent.width) && rh == static_cast<int>(extent.height);
    const std::size_t count = static_cast<std::size_t>(extent.width) * extent.height;
    std::uint64_t different = 0;
    double sumSquared = 0.0;
    for (std::size_t i = 0; i < count && sameSize; ++i) {
        int pixelDiff = 0;
        for (int c = 0; c < 3; ++c) {
            const int a = pixels[i * 4 + static_cast<std::size_t>(c)];
            const int b = ref[i * 4 + static_cast<std::size_t>(c)];
            pixelDiff = std::max(pixelDiff, std::abs(a - b));
            const double d = static_cast<double>(a - b);
            sumSquared += d * d;
        }
        if (pixelDiff > 8) {
            ++different;
        }
    }
    stbi_image_free(ref);

    if (!sameSize) {
        std::fprintf(stderr, "[headless] golden size mismatch: reference %dx%d, rendered %ux%u\n", rw, rh, extent.width,
                     extent.height);
        diffPercent = 100.0;
        psnr = 0.0;
        return false;
    }

    diffPercent = 100.0 * static_cast<double>(different) / static_cast<double>(count);
    const double mse = sumSquared / static_cast<double>(count * 3);
    psnr = mse <= 0.0 ? 100.0 : 10.0 * std::log10((255.0 * 255.0) / mse);

    std::printf("[headless] golden: diff=%.3f%% (max %.3f%%), PSNR=%.2f dB (min %.2f dB)\n", diffPercent,
                maxDiffPercent, psnr, minPsnr);
    return diffPercent <= maxDiffPercent && psnr >= minPsnr;
}

// ---- Semantic (projected pixel) checks -------------------------------------

bool checkSemantics(const render::Scene& scene, const std::vector<SceneProbe>& probes,
                    const std::vector<std::uint8_t>& pixels, rhi::Extent2D extent) {
    const glm::mat4 vp = scene.camera().viewProjection();
    bool ok = true;
    for (const SceneProbe& probe : probes) {
        const glm::vec4 clip = vp * glm::vec4(probe.world, 1.0f);
        if (clip.w <= 0.0f) {
            std::fprintf(stderr, "[headless] semantic: probe (%.2f,%.2f,%.2f) is behind the camera\n", probe.world.x,
                         probe.world.y, probe.world.z);
            ok = false;
            continue;
        }
        const float ndcX = clip.x / clip.w;
        const float ndcY = clip.y / clip.w;
        const int px = static_cast<int>((ndcX * 0.5f + 0.5f) * static_cast<float>(extent.width));
        const int py = static_cast<int>((1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(extent.height));
        if (px < 0 || py < 0 || px >= static_cast<int>(extent.width) || py >= static_cast<int>(extent.height)) {
            std::fprintf(stderr, "[headless] semantic: probe (%.2f,%.2f,%.2f) projects offscreen at (%d,%d)\n",
                         probe.world.x, probe.world.y, probe.world.z, px, py);
            ok = false;
            continue;
        }

        // Compare chromaticity (hue) rather than absolute colour: the Lambert
        // shading darkens surfaces by an unknown factor that depends on the
        // face normal, but the ratio between channels is stable. Search a small
        // neighbourhood so sub-pixel projection and edge cases do not flake.
        const auto chromaticity = [](float r, float g, float b) {
            const float sum = r + g + b + 1e-5f;
            return glm::vec3(r / sum, g / sum, b / sum);
        };
        const glm::vec3 expected = chromaticity(probe.color.r, probe.color.g, probe.color.b);
        const float tol = 0.10f;

        bool match = false;
        float bestError = 1e9f;
        glm::vec3 bestColor{0.0f};
        for (int dy = -8; dy <= 8 && !match; ++dy) {
            for (int dx = -8; dx <= 8 && !match; ++dx) {
                const int sx = px + dx;
                const int sy = py + dy;
                if (sx < 0 || sy < 0 || sx >= static_cast<int>(extent.width) || sy >= static_cast<int>(extent.height)) {
                    continue;
                }
                const std::size_t index =
                    (static_cast<std::size_t>(sy) * extent.width + static_cast<std::size_t>(sx)) * 4;
                const glm::vec3 c =
                    chromaticity(pixels[index + 0] / 255.0f, pixels[index + 1] / 255.0f, pixels[index + 2] / 255.0f);
                const float error =
                    std::max({std::abs(c.x - expected.x), std::abs(c.y - expected.y), std::abs(c.z - expected.z)});
                if (error < bestError) {
                    bestError = error;
                    bestColor = c;
                }
                if (error < tol) {
                    match = true;
                }
            }
        }
        std::printf("[headless] semantic: (%.2f,%.2f,%.2f) -> pixel (%d,%d) best chromaticity (%.2f,%.2f,%.2f) vs "
                    "(%.2f,%.2f,%.2f) err %.3f %s\n",
                    probe.world.x, probe.world.y, probe.world.z, px, py, bestColor.x, bestColor.y, bestColor.z,
                    expected.x, expected.y, expected.z, bestError, match ? "ok" : "FAIL");
        if (!match) {
            ok = false;
        }
    }
    return ok;
}

// Absolute orientation checks for the "orientation" scene that never touch the
// projection matrix: they only look at pixels, so a wrong matrix convention
// cannot make them agree with a wrong image.
//
//  1. Horizon: with a horizontal camera the top row is sky and the bottom row
//     is ground.
//  2. Placement: the pixel centroid of the box that is higher in the world is
//     higher in the image, and the +X box is right of the -X box.
//  3. Culling: the "high" box sits entirely above the eye, so only its front
//     (+Z) and bottom (-Y) faces may be visible. The bottom face only gets the
//     ground ambient and is darker than the sun-lit front face, so the lowest
//     rows of its silhouette must be darker than the middle ones. With an
//     inverted front face the interior shows instead: a bright +Y strip on top
//     over a uniformly dark back face.
bool checkOrientation(const render::Scene& scene, const std::vector<std::uint8_t>& pixels, rhi::Extent2D extent) {
    const int width = static_cast<int>(extent.width);
    const int height = static_cast<int>(extent.height);
    const auto pixel = [&](int x, int y) {
        const std::size_t i = (static_cast<std::size_t>(y) * extent.width + static_cast<std::size_t>(x)) * 4;
        return glm::vec3(pixels[i] / 255.0f, pixels[i + 1] / 255.0f, pixels[i + 2] / 255.0f);
    };
    bool ok = true;

    // 1. Horizon, sampled along the centre column.
    const glm::vec3 sky(scene.skyColor());
    const glm::vec3 top = pixel(width / 2, height / 20);
    const glm::vec3 bottom = pixel(width / 2, height - 1 - height / 20);
    const bool horizonOk = glm::length(top - sky) < 0.05f && glm::length(bottom - sky) > 0.15f;
    std::printf(
        "[headless] orientation horizon: top=(%.3f,%.3f,%.3f) sky=(%.3f,%.3f,%.3f) bottom=(%.3f,%.3f,%.3f) %s\n", top.r,
        top.g, top.b, sky.r, sky.g, sky.b, bottom.r, bottom.g, bottom.b, horizonOk ? "ok" : "FAIL");
    ok = ok && horizonOk;

    // Pixels of a box, found by chromaticity (shading scales all channels).
    struct Blob {
        double sumX = 0.0;
        double sumY = 0.0;
        int count = 0;
        int minY = 1 << 30;
        int maxY = -1;
        glm::vec2 centroid() const { return {static_cast<float>(sumX / count), static_cast<float>(sumY / count)}; }
    };
    const auto chroma = [](const glm::vec3& c) { return c / (c.r + c.g + c.b + 1e-5f); };
    const auto matches = [&](const glm::vec3& c, const glm::vec4& base) {
        const glm::vec3 d = glm::abs(chroma(c) - chroma(glm::vec3(base)));
        return std::max({d.x, d.y, d.z}) < 0.06f && (c.r + c.g + c.b) > 0.03f;
    };
    const auto findBlob = [&](const glm::vec4& base) {
        Blob blob;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (matches(pixel(x, y), base)) {
                    blob.sumX += x;
                    blob.sumY += y;
                    ++blob.count;
                    blob.minY = std::min(blob.minY, y);
                    blob.maxY = std::max(blob.maxY, y);
                }
            }
        }
        return blob;
    };

    const std::vector<SceneProbe> boxes = sceneProbes("orientation"); // reference, high, left, right
    const Blob reference = findBlob(boxes[0].color);
    const Blob high = findBlob(boxes[1].color);
    const Blob left = findBlob(boxes[2].color);
    const Blob right = findBlob(boxes[3].color);
    if (reference.count < 50 || high.count < 50 || left.count < 50 || right.count < 50) {
        std::fprintf(stderr, "[headless] orientation: a box is missing (pixels ref=%d high=%d left=%d right=%d)\n",
                     reference.count, high.count, left.count, right.count);
        return false;
    }

    // 2. Placement.
    const bool aboveOk = high.centroid().y < reference.centroid().y;
    const bool rightOk = right.centroid().x > left.centroid().x;
    std::printf("[headless] orientation placement: high y=%.1f < reference y=%.1f %s, right x=%.1f > left x=%.1f %s\n",
                high.centroid().y, reference.centroid().y, aboveOk ? "ok" : "FAIL", right.centroid().x,
                left.centroid().x, rightOk ? "ok" : "FAIL");
    ok = ok && aboveOk && rightOk;

    // 3. Culling, from the mean luminance of horizontal bands of the high box.
    const auto bandLuminance = [&](float from, float to) {
        const int span = high.maxY - high.minY + 1;
        const int y0 = high.minY + static_cast<int>(from * static_cast<float>(span));
        const int y1 = high.minY + static_cast<int>(to * static_cast<float>(span));
        double sum = 0.0;
        int count = 0;
        for (int y = y0; y < y1; ++y) {
            for (int x = 0; x < width; ++x) {
                const glm::vec3 c = pixel(x, y);
                if (matches(c, boxes[1].color)) {
                    sum += 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b;
                    ++count;
                }
            }
        }
        return count > 0 ? static_cast<float>(sum / count) : 0.0f;
    };
    const float topBand = bandLuminance(0.0f, 0.15f);
    const float middleBand = bandLuminance(0.40f, 0.60f);
    const float bottomBand = bandLuminance(0.88f, 1.0f);
    const bool cullingOk = bottomBand < 0.85f * middleBand && topBand < 1.10f * middleBand;
    std::printf("[headless] orientation culling: high box luminance top=%.3f middle=%.3f bottom=%.3f %s\n", topBand,
                middleBand, bottomBand, cullingOk ? "ok" : "FAIL");
    return ok && cullingOk;
}

// ---- Contact sheet ---------------------------------------------------------

void fillRect(std::vector<std::uint8_t>& image, int width, int height, int x0, int y0, int x1, int y1, std::uint8_t r,
              std::uint8_t g, std::uint8_t b, std::uint8_t a) {
    x0 = std::max(x0, 0);
    y0 = std::max(y0, 0);
    x1 = std::min(x1, width);
    y1 = std::min(y1, height);
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            const std::size_t i =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)) * 4;
            const float alpha = a / 255.0f;
            image[i + 0] = static_cast<std::uint8_t>(image[i + 0] * (1.0f - alpha) + r * alpha);
            image[i + 1] = static_cast<std::uint8_t>(image[i + 1] * (1.0f - alpha) + g * alpha);
            image[i + 2] = static_cast<std::uint8_t>(image[i + 2] * (1.0f - alpha) + b * alpha);
            image[i + 3] = 255;
        }
    }
}

// Draws white text using stb_easy_font, scaled by `scale`.
void drawText(std::vector<std::uint8_t>& image, int width, int height, int originX, int originY,
              const std::string& text, float scale) {
    static char buffer[65536];
    const int quads = stb_easy_font_print(0.0f, 0.0f, const_cast<char*>(text.c_str()), nullptr, buffer, sizeof(buffer));
    const float* verts = reinterpret_cast<const float*>(buffer);
    for (int q = 0; q < quads; ++q) {
        float xs[4];
        float ys[4];
        for (int k = 0; k < 4; ++k) {
            xs[k] = verts[q * 16 + k * 4 + 0];
            ys[k] = verts[q * 16 + k * 4 + 1];
        }
        const float minX = *std::min_element(xs, xs + 4);
        const float maxX = *std::max_element(xs, xs + 4);
        const float minY = *std::min_element(ys, ys + 4);
        const float maxY = *std::max_element(ys, ys + 4);
        fillRect(image, width, height, originX + static_cast<int>(minX * scale),
                 originY + static_cast<int>(minY * scale), originX + static_cast<int>(maxX * scale) + 1,
                 originY + static_cast<int>(maxY * scale) + 1, 255, 255, 255, 255);
    }
}

int runContactSheet(rhi::Device& device, vfs::VirtualFileSystem& vfs, render::Renderer& renderer,
                    const std::shared_ptr<render::Mesh>& cube, const std::shared_ptr<render::Mesh>& sphere,
                    const std::shared_ptr<render::Mesh>& cylinder, const std::shared_ptr<render::Mesh>& ground,
                    const std::vector<std::string>& names, const std::filesystem::path& output) {
    const int tileW = 480;
    const int tileH = 270;
    const int labelH = 26;
    const int columns = 2;
    const int rows = (static_cast<int>(names.size()) + columns - 1) / columns;
    const int sheetW = tileW * columns;
    const int sheetH = (tileH + labelH) * rows;
    std::vector<std::uint8_t> sheet(static_cast<std::size_t>(sheetW) * sheetH * 4);
    // Opaque dark background, so unused tiles do not show up as transparent.
    for (std::size_t i = 0; i < sheet.size(); i += 4) {
        sheet[i + 0] = 22;
        sheet[i + 1] = 24;
        sheet[i + 2] = 30;
        sheet[i + 3] = 255;
    }

    Offscreen target;
    createTargets(device, target, {static_cast<std::uint32_t>(tileW), static_cast<std::uint32_t>(tileH)});
    renderer.ensureTargets(target.extent, kColorFormat);

    std::unique_ptr<render::DebugDraw> debug;
    debug = std::make_unique<render::DebugDraw>(device, vfs);
    debug->ensureTargets(kColorFormat, target.extent);

    render::Scene scene;
    for (std::size_t n = 0; n < names.size(); ++n) {
        std::unique_ptr<physics::PhysicsWorld> world;
        const bool physicsScene = isPhysicsScene(names[n]);
        if (physicsScene) {
            world = std::make_unique<physics::PhysicsWorld>();
            const double settle = buildPhysicsWorld(names[n], *world);
            const int steps = static_cast<int>(settle / world->fixedStep + 0.5);
            for (int step = 0; step < steps; ++step) {
                world->step(world->fixedStep);
            }
            buildSceneFromWorld(*world, scene, cube, sphere, cylinder, ground);
            physicsSceneCamera(names[n], scene, static_cast<float>(tileW) / static_cast<float>(tileH));
        } else {
            buildScene(names[n], scene, cube, ground);
        }

        const std::vector<std::uint8_t> pixels = [&]() {
            if (physicsScene) {
                const auto decorate = [&](render::DebugDraw& draw) { drawPhysicsSceneDebug(names[n], *world, draw); };
                return renderFrames(device, renderer, target, scene, 1, -1, false, debug.get(), decorate);
            }
            return renderFrames(device, renderer, target, scene, 1, -1, false);
        }();

        const int cell = static_cast<int>(n);
        const int col = cell % columns;
        const int row = cell / columns;
        const int ox = col * tileW;
        const int oy = row * (tileH + labelH);

        for (int y = 0; y < tileH; ++y) {
            std::memcpy(&sheet[(static_cast<std::size_t>(oy + y) * sheetW + ox) * 4],
                        &pixels[static_cast<std::size_t>(y) * tileW * 4], static_cast<std::size_t>(tileW) * 4);
        }

        fillRect(sheet, sheetW, sheetH, ox, oy + tileH, ox + tileW, oy + tileH + labelH, 20, 22, 28, 255);
        drawText(sheet, sheetW, sheetH, ox + 8, oy + tileH + 6, names[n], 2.0f);
        std::printf("[headless] contact sheet tile %zu: %s\n", n, names[n].c_str());
    }

    if (!writePng(output, sheet, {static_cast<std::uint32_t>(sheetW), static_cast<std::uint32_t>(sheetH)})) {
        std::fprintf(stderr, "[headless] failed to write contact sheet %s\n", output.string().c_str());
        return 1;
    }
    std::printf("[headless] wrote contact sheet %s (%dx%d)\n", output.string().c_str(), sheetW, sheetH);
    return 0;
}

} // namespace

int runHeadless(const HeadlessOptions& options) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);
    const std::filesystem::path exeDir = options.executablePath.empty()
                                             ? std::filesystem::current_path()
                                             : std::filesystem::absolute(options.executablePath).parent_path();
    const std::filesystem::path root = searchForRoot(exeDir);

    core::Log::Config logConfig;
    logConfig.appName = "sim-headless";
    logConfig.logFile = root / "sim-headless.log";
    core::Log::init(logConfig);
    auto log = core::Log::get();

    log->info("Car Simulator {} headless: {} frame(s), {}x{}, scene '{}'", core::kVersionString, options.frames,
              options.width, options.height, options.scene);
    std::printf("[headless] root: %s\n", root.string().c_str());
    std::printf("[headless] rendering %u frame(s) at %ux%u (scene '%s')\n", options.frames, options.width,
                options.height, options.scene.c_str());

    vfs::VirtualFileSystem vfs;
    vfs.mount(exeDir, "/", true);

    // Mount the official content so the drive scene can load the real car and
    // circuit through the same path the game uses. Only base content is mounted
    // here, so every descriptor belongs to `base.core`.
    if (std::filesystem::exists(root / "content" / "base")) {
        vfs.mount(root / "content" / "base", "/", false);
    }
    const ContentOwner owner = [](std::string_view) { return std::string(modding::kBaseModId); };
    CarCatalog::instance().reload(vfs, owner);
    TrackCatalog::instance().reload(vfs, owner);

    rhi::DeviceConfig deviceConfig;
    deviceConfig.headless = true;
    deviceConfig.enableValidation = options.validation;
    auto device = std::make_unique<rhi::Device>(deviceConfig);
    log->info("device: {} (headless)", device->deviceName());
    std::printf("[headless] device: %s\n", device->deviceName().c_str());

    render::Renderer renderer(*device, vfs, 1);

    auto cube = render::Mesh::makeCube(*device);
    auto sphere = render::Mesh::makeSphere(*device);
    auto cylinder = render::Mesh::makeCylinder(*device);
    auto ground = render::Mesh::makeGroundPlane(*device, 200.0f);

    int result = 0;

    if (!options.contactSheet.empty()) {
        result = runContactSheet(*device, vfs, renderer, cube, sphere, cylinder, ground, physicsSceneNames(),
                                 options.contactSheet);
    }
    if (result == 0 && !options.renderTestsSheet.empty()) {
        result = runContactSheet(*device, vfs, renderer, cube, sphere, cylinder, ground, renderTestSceneNames(),
                                 options.renderTestsSheet);
    } else {
        Offscreen target;
        createTargets(*device, target, {options.width, options.height});
        renderer.ensureTargets(target.extent, kColorFormat);
        if (!renderer.ready()) {
            log->critical("renderer is not ready (missing shaders?)");
            std::fprintf(stderr, "[headless] renderer not ready\n");
            device->waitIdle();
            return 1;
        }

        render::Scene scene;
        std::unique_ptr<physics::PhysicsWorld> world;
        std::unique_ptr<drive::DriveScene> driveScene;
        const bool physicsScene = isPhysicsScene(options.scene);
        if (drive::isDriveScene(options.scene)) {
            // The drivable circuit: let the car settle on its springs at the
            // start line, optionally autopilot for `time` seconds (so a capture
            // can show the car moving or cornering), then frame it. The car and
            // circuit come from the content that was just mounted.
            const CarSpec* car = CarCatalog::instance().defaultCar();
            const TrackSpec* track = TrackCatalog::instance().defaultTrack();
            static const drive::TrackDescription kReferenceTrack = drive::defaultTrackDescription();
            driveScene =
                std::make_unique<drive::DriveScene>(*device, track != nullptr ? track->track : kReferenceTrack,
                                                    car != nullptr ? car->setup : vehicle::makeDefaultVehicleSetup());
            driveScene->simulate(1.0);
            if (options.time > 0.0) {
                driveScene->autopilot(options.time);
            }
            driveScene->setOrbit(options.yaw, options.pitch);
            driveScene->setCamera(driveCameraFromName(options.camera));
            driveScene->snapCamera();
            const float aspect = static_cast<float>(options.width) / static_cast<float>(std::max(options.height, 1u));
            driveScene->buildScene(scene, aspect, 0.0);
        } else if (physicsScene) {
            world = std::make_unique<physics::PhysicsWorld>();
            const double settle = buildPhysicsWorld(options.scene, *world);
            const int steps = static_cast<int>(settle / world->fixedStep + 0.5);
            for (int step = 0; step < steps; ++step) {
                world->step(world->fixedStep);
            }
            buildSceneFromWorld(*world, scene, cube, sphere, cylinder, ground);
            const float aspect = static_cast<float>(options.width) / static_cast<float>(std::max(options.height, 1u));
            physicsSceneCamera(options.scene, scene, aspect);
        } else {
            buildScene(options.scene, scene, cube, ground);
        }

        // Physics scenes always draw the overlay (grid + axes + scene-specific
        // lines: analytic trajectories, joints, springs, body axes), so their
        // golden references pin both the physics state and the line overlay.
        const bool useDebug = options.debugDraw || physicsScene;
        std::unique_ptr<render::DebugDraw> debug;
        if (useDebug) {
            debug = std::make_unique<render::DebugDraw>(*device, vfs);
            debug->ensureTargets(kColorFormat, target.extent);
            if (!debug->ready()) {
                std::fprintf(stderr, "[headless] debug draw pipeline unavailable\n");
            }
        }

        const std::uint32_t frames = options.frames == 0 ? 1 : options.frames;
        const auto decorate = [&](render::DebugDraw& draw) {
            if (physicsScene) {
                drawPhysicsSceneDebug(options.scene, *world, draw);
            }
            if (driveScene != nullptr) {
                driveScene->drawDebug(draw);
            }
        };
        const std::vector<std::uint8_t> pixels =
            renderFrames(*device, renderer, target, scene, frames, options.resizeFrame, true, debug.get(), decorate);

        if (!options.screenshot.empty()) {
            if (writePng(options.screenshot, pixels, target.extent)) {
                std::printf("[headless] wrote %s (%ux%u)\n", options.screenshot.string().c_str(), target.extent.width,
                            target.extent.height);
            } else {
                std::fprintf(stderr, "[headless] failed to write %s\n", options.screenshot.string().c_str());
                result = 1;
            }
        }

        if (options.semantic && result == 0) {
            if (!checkSemantics(scene, sceneProbes(options.scene), pixels, target.extent)) {
                result = 4;
            } else if (options.scene == "orientation" && !checkOrientation(scene, pixels, target.extent)) {
                result = 4;
            }
        }

        if (options.verifyGolden || options.updateGolden) {
            const std::string goldenName = options.scene + (options.debugDraw ? "_debug" : "");
            std::filesystem::path reference = options.goldenDir / (goldenName + ".png");
            if (options.updateGolden) {
                std::filesystem::create_directories(options.goldenDir);
                if (writePng(reference, pixels, target.extent)) {
                    std::printf("[headless] updated golden %s (%ux%u)\n", reference.string().c_str(),
                                target.extent.width, target.extent.height);
                } else {
                    std::fprintf(stderr, "[headless] failed to write golden %s\n", reference.string().c_str());
                    result = 1;
                }
            } else if (result == 0) {
                if (!std::filesystem::exists(reference)) {
                    std::printf("[headless] golden %s not present (not approved yet); verification skipped\n",
                                reference.string().c_str());
                    return 77; // ctest maps this exit code to SKIPPED
                }
                double diffPercent = 0.0;
                double psnr = 0.0;
                if (!compareGolden(reference, pixels, target.extent, options.maxDiffPercent, options.minPsnr,
                                   diffPercent, psnr)) {
                    std::fprintf(stderr, "[headless] golden mismatch for scene '%s'\n", options.scene.c_str());
                    result = 3;
                }
            }
        }
    }

    device->waitIdle();

    const std::uint32_t messages = device->validationMessageCount();
    if (messages > 0) {
        std::fprintf(stderr, "[headless] %u Vulkan validation message(s) emitted\n", messages);
    }

    if (result != 0) {
        return result;
    }
    return messages > 0 ? 2 : 0;
}

} // namespace sim::app
