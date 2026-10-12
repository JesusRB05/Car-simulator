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
#include <memory>
#include <stdexcept>
#include <vector>

#include "Scenes.hpp"
#include "core/Log.hpp"
#include "core/Version.hpp"
#include "render/DebugDraw.hpp"
#include "render/Mesh.hpp"
#include "render/Renderer.hpp"
#include "render/Scene.hpp"
#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"
#include "rhi/Resources.hpp"
#include "rhi/Types.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

constexpr rhi::Format kColorFormat = rhi::Format::R8G8B8A8Unorm;
constexpr float kClear[4] = {0.05f, 0.06f, 0.08f, 1.0f};

std::filesystem::path searchForRoot(const std::filesystem::path& exeDir)
{
    std::vector<std::filesystem::path> starts{std::filesystem::current_path(), exeDir};
    for (const std::filesystem::path& start : starts)
    {
        std::filesystem::path dir = start;
        for (int depth = 0; depth < 8 && !dir.empty(); ++depth)
        {
            if (std::filesystem::exists(dir / "content" / "base"))
            {
                return dir;
            }
            const std::filesystem::path parent = dir.parent_path();
            if (parent == dir)
            {
                break;
            }
            dir = parent;
        }
    }
    return exeDir;
}

void submitAndWait(rhi::Device& device, rhi::CommandList& commandList)
{
    VkFence fence = VK_NULL_HANDLE;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (vkCreateFence(device.handle(), &fenceInfo, nullptr, &fence) != VK_SUCCESS)
    {
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

struct Offscreen
{
    rhi::Extent2D extent{};
    std::unique_ptr<rhi::Image> color;
    std::unique_ptr<rhi::Buffer> readback;
};

void createTargets(rhi::Device& device, Offscreen& target, rhi::Extent2D extent)
{
    target.extent = extent;
    target.color = std::make_unique<rhi::Image>(device, extent, kColorFormat,
                                                rhi::ImageUsage::ColorAttachment | rhi::ImageUsage::TransferSrc);
    const std::size_t size = static_cast<std::size_t>(extent.width) * extent.height * 4;
    target.readback = std::make_unique<rhi::Buffer>(device, size, rhi::BufferUsage::TransferDst, rhi::MemoryUsage::GpuToCpu);
}

// Renders `frames` frames into the offscreen target and returns the last frame's
// RGBA pixels (top-left origin, matching the PNG writer).
std::vector<std::uint8_t> renderFrames(rhi::Device& device, render::Renderer& renderer, Offscreen& target, render::Scene& scene,
                                       std::uint32_t frames, int resizeFrame, bool logResize,
                                       render::DebugDraw* debug = nullptr)
{
    rhi::CommandList commandList(device);
    for (std::uint32_t frame = 0; frame < frames; ++frame)
    {
        if (resizeFrame >= 0 && frame == static_cast<std::uint32_t>(resizeFrame))
        {
            device.waitIdle();
            target.color.reset();
            target.readback.reset();
            const rhi::Extent2D smaller{std::max(target.extent.width / 2, 1u), std::max(target.extent.height / 2, 1u)};
            createTargets(device, target, smaller);
            renderer.ensureTargets(target.extent, kColorFormat);
            if (logResize)
            {
                std::printf("[headless] resized offscreen target to %ux%u at frame %u\n", target.extent.width, target.extent.height,
                            frame);
            }
        }

        const bool last = frame + 1 == frames;

        commandList.begin();
        commandList.transitionImage(target.color->handle(), rhi::ImageLayout::Undefined, rhi::ImageLayout::ColorAttachment);

        rhi::ColorAttachment attachment;
        attachment.image = reinterpret_cast<std::uint64_t>(target.color->view());
        attachment.layout = rhi::ImageLayout::ColorAttachment;
        attachment.loadOp = rhi::LoadOp::Clear;
        std::memcpy(attachment.clearColor, kClear, sizeof(kClear));

        renderer.updateFrameUniforms(0, scene.camera(), scene.lightDirection(), scene.lightColor());
        renderer.renderScene(commandList, scene, attachment, 0);

        if (debug != nullptr)
        {
            rhi::ColorAttachment overlay = attachment;
            overlay.loadOp = rhi::LoadOp::Load;
            debug->clear();
            debug->addGrid(20.0f, 1.0f, {0.55f, 0.58f, 0.62f, 1.0f}, {0.75f, 0.30f, 0.20f, 1.0f});
            debug->addAxes({0.0f, 0.01f, 0.0f}, 3.0f);
            debug->render(commandList, scene.camera(), overlay);
        }

        if (last)
        {
            commandList.transitionImage(target.color->handle(), rhi::ImageLayout::ColorAttachment, rhi::ImageLayout::TransferSrc);
            commandList.copyImageToBuffer(target.color->handle(), target.readback->handle(), target.extent);
        }
        commandList.end();
        submitAndWait(device, commandList);
    }

    const std::size_t size = static_cast<std::size_t>(target.extent.width) * target.extent.height * 4;
    std::vector<std::uint8_t> pixels(size);
    if (const void* mapped = target.readback->mappedData())
    {
        std::memcpy(pixels.data(), mapped, size);
    }
    return pixels;
}

// ---- PNG helpers -----------------------------------------------------------

bool writePng(const std::filesystem::path& path, const std::vector<std::uint8_t>& pixels, rhi::Extent2D extent)
{
    if (!path.parent_path().empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
    }
    const int stride = static_cast<int>(extent.width) * 4;
    const int written = stbi_write_png(path.string().c_str(), static_cast<int>(extent.width), static_cast<int>(extent.height), 4,
                                       pixels.data(), stride);
    return written != 0;
}

// Compares `pixels` against the reference PNG. Returns true when the image is
// within tolerance; fills the measured metrics for reporting.
bool compareGolden(const std::filesystem::path& reference, const std::vector<std::uint8_t>& pixels, rhi::Extent2D extent,
                   double maxDiffPercent, double minPsnr, double& diffPercent, double& psnr)
{
    int rw = 0;
    int rh = 0;
    int channels = 0;
    unsigned char* ref = stbi_load(reference.string().c_str(), &rw, &rh, &channels, 4);
    if (ref == nullptr)
    {
        std::fprintf(stderr, "[headless] cannot read golden reference %s\n", reference.string().c_str());
        return false;
    }

    const bool sameSize = rw == static_cast<int>(extent.width) && rh == static_cast<int>(extent.height);
    const std::size_t count = static_cast<std::size_t>(extent.width) * extent.height;
    std::uint64_t different = 0;
    double sumSquared = 0.0;
    for (std::size_t i = 0; i < count && sameSize; ++i)
    {
        int pixelDiff = 0;
        for (int c = 0; c < 3; ++c)
        {
            const int a = pixels[i * 4 + static_cast<std::size_t>(c)];
            const int b = ref[i * 4 + static_cast<std::size_t>(c)];
            pixelDiff = std::max(pixelDiff, std::abs(a - b));
            const double d = static_cast<double>(a - b);
            sumSquared += d * d;
        }
        if (pixelDiff > 8)
        {
            ++different;
        }
    }
    stbi_image_free(ref);

    if (!sameSize)
    {
        std::fprintf(stderr, "[headless] golden size mismatch: reference %dx%d, rendered %ux%u\n", rw, rh, extent.width,
                     extent.height);
        diffPercent = 100.0;
        psnr = 0.0;
        return false;
    }

    diffPercent = 100.0 * static_cast<double>(different) / static_cast<double>(count);
    const double mse = sumSquared / static_cast<double>(count * 3);
    psnr = mse <= 0.0 ? 100.0 : 10.0 * std::log10((255.0 * 255.0) / mse);

    std::printf("[headless] golden: diff=%.3f%% (max %.3f%%), PSNR=%.2f dB (min %.2f dB)\n", diffPercent, maxDiffPercent, psnr,
                minPsnr);
    return diffPercent <= maxDiffPercent && psnr >= minPsnr;
}

// ---- Semantic (projected pixel) checks -------------------------------------

bool checkSemantics(const render::Scene& scene, const std::vector<SceneProbe>& probes, const std::vector<std::uint8_t>& pixels,
                    rhi::Extent2D extent)
{
    const glm::mat4 vp = scene.camera().viewProjection();
    bool ok = true;
    for (const SceneProbe& probe : probes)
    {
        const glm::vec4 clip = vp * glm::vec4(probe.world, 1.0f);
        if (clip.w <= 0.0f)
        {
            std::fprintf(stderr, "[headless] semantic: probe (%.2f,%.2f,%.2f) is behind the camera\n", probe.world.x, probe.world.y,
                         probe.world.z);
            ok = false;
            continue;
        }
        const float ndcX = clip.x / clip.w;
        const float ndcY = clip.y / clip.w;
        const int px = static_cast<int>((ndcX * 0.5f + 0.5f) * static_cast<float>(extent.width));
        const int py = static_cast<int>((1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(extent.height));
        if (px < 0 || py < 0 || px >= static_cast<int>(extent.width) || py >= static_cast<int>(extent.height))
        {
            std::fprintf(stderr, "[headless] semantic: probe (%.2f,%.2f,%.2f) projects offscreen at (%d,%d)\n", probe.world.x,
                         probe.world.y, probe.world.z, px, py);
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
        for (int dy = -8; dy <= 8 && !match; ++dy)
        {
            for (int dx = -8; dx <= 8 && !match; ++dx)
            {
                const int sx = px + dx;
                const int sy = py + dy;
                if (sx < 0 || sy < 0 || sx >= static_cast<int>(extent.width) || sy >= static_cast<int>(extent.height))
                {
                    continue;
                }
                const std::size_t index = (static_cast<std::size_t>(sy) * extent.width + static_cast<std::size_t>(sx)) * 4;
                const glm::vec3 c = chromaticity(pixels[index + 0] / 255.0f, pixels[index + 1] / 255.0f, pixels[index + 2] / 255.0f);
                const float error = std::max({std::abs(c.x - expected.x), std::abs(c.y - expected.y), std::abs(c.z - expected.z)});
                if (error < bestError)
                {
                    bestError = error;
                    bestColor = c;
                }
                if (error < tol)
                {
                    match = true;
                }
            }
        }
        std::printf("[headless] semantic: (%.2f,%.2f,%.2f) -> pixel (%d,%d) best chromaticity (%.2f,%.2f,%.2f) vs (%.2f,%.2f,%.2f) err %.3f %s\n",
                    probe.world.x, probe.world.y, probe.world.z, px, py, bestColor.x, bestColor.y, bestColor.z, expected.x, expected.y,
                    expected.z, bestError, match ? "ok" : "FAIL");
        if (!match)
        {
            ok = false;
        }
    }
    return ok;
}

// ---- Contact sheet ---------------------------------------------------------

void fillRect(std::vector<std::uint8_t>& image, int width, int height, int x0, int y0, int x1, int y1, std::uint8_t r, std::uint8_t g,
              std::uint8_t b, std::uint8_t a)
{
    x0 = std::max(x0, 0);
    y0 = std::max(y0, 0);
    x1 = std::min(x1, width);
    y1 = std::min(y1, height);
    for (int y = y0; y < y1; ++y)
    {
        for (int x = x0; x < x1; ++x)
        {
            const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)) * 4;
            const float alpha = a / 255.0f;
            image[i + 0] = static_cast<std::uint8_t>(image[i + 0] * (1.0f - alpha) + r * alpha);
            image[i + 1] = static_cast<std::uint8_t>(image[i + 1] * (1.0f - alpha) + g * alpha);
            image[i + 2] = static_cast<std::uint8_t>(image[i + 2] * (1.0f - alpha) + b * alpha);
            image[i + 3] = 255;
        }
    }
}

// Draws white text using stb_easy_font, scaled by `scale`.
void drawText(std::vector<std::uint8_t>& image, int width, int height, int originX, int originY, const std::string& text, float scale)
{
    static char buffer[65536];
    const int quads = stb_easy_font_print(0.0f, 0.0f, const_cast<char*>(text.c_str()), nullptr, buffer, sizeof(buffer));
    const float* verts = reinterpret_cast<const float*>(buffer);
    for (int q = 0; q < quads; ++q)
    {
        float xs[4];
        float ys[4];
        for (int k = 0; k < 4; ++k)
        {
            xs[k] = verts[q * 16 + k * 4 + 0];
            ys[k] = verts[q * 16 + k * 4 + 1];
        }
        const float minX = *std::min_element(xs, xs + 4);
        const float maxX = *std::max_element(xs, xs + 4);
        const float minY = *std::min_element(ys, ys + 4);
        const float maxY = *std::max_element(ys, ys + 4);
        fillRect(image, width, height, originX + static_cast<int>(minX * scale), originY + static_cast<int>(minY * scale),
                 originX + static_cast<int>(maxX * scale) + 1, originY + static_cast<int>(maxY * scale) + 1, 255, 255, 255, 255);
    }
}

int runContactSheet(rhi::Device& device, render::Renderer& renderer, const std::shared_ptr<render::Mesh>& cube,
                    const std::shared_ptr<render::Mesh>& ground, const std::filesystem::path& output)
{
    const std::vector<std::string> names = sceneNames();
    const int tileW = 480;
    const int tileH = 270;
    const int labelH = 26;
    const int columns = 2;
    const int rows = (static_cast<int>(names.size()) + columns - 1) / columns;
    const int sheetW = tileW * columns;
    const int sheetH = (tileH + labelH) * rows;
    std::vector<std::uint8_t> sheet(static_cast<std::size_t>(sheetW) * sheetH * 4);

    Offscreen target;
    createTargets(device, target, {static_cast<std::uint32_t>(tileW), static_cast<std::uint32_t>(tileH)});
    renderer.ensureTargets(target.extent, kColorFormat);

    render::Scene scene;
    for (std::size_t n = 0; n < names.size(); ++n)
    {
        buildScene(names[n], scene, cube, ground);
        const std::vector<std::uint8_t> pixels = renderFrames(device, renderer, target, scene, 1, -1, false);

        const int cell = static_cast<int>(n);
        const int col = cell % columns;
        const int row = cell / columns;
        const int ox = col * tileW;
        const int oy = row * (tileH + labelH);

        for (int y = 0; y < tileH; ++y)
        {
            std::memcpy(&sheet[(static_cast<std::size_t>(oy + y) * sheetW + ox) * 4],
                        &pixels[static_cast<std::size_t>(y) * tileW * 4], static_cast<std::size_t>(tileW) * 4);
        }

        fillRect(sheet, sheetW, sheetH, ox, oy + tileH, ox + tileW, oy + tileH + labelH, 20, 22, 28, 255);
        drawText(sheet, sheetW, sheetH, ox + 8, oy + tileH + 6, names[n], 2.0f);
        std::printf("[headless] contact sheet tile %zu: %s\n", n, names[n].c_str());
    }

    if (!writePng(output, sheet, {static_cast<std::uint32_t>(sheetW), static_cast<std::uint32_t>(sheetH)}))
    {
        std::fprintf(stderr, "[headless] failed to write contact sheet %s\n", output.string().c_str());
        return 1;
    }
    std::printf("[headless] wrote contact sheet %s (%dx%d)\n", output.string().c_str(), sheetW, sheetH);
    return 0;
}

} // namespace

int runHeadless(const HeadlessOptions& options)
{
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

    log->info("Car Simulator {} headless: {} frame(s), {}x{}, scene '{}'", core::kVersionString, options.frames, options.width,
              options.height, options.scene);
    std::printf("[headless] root: %s\n", root.string().c_str());
    std::printf("[headless] rendering %u frame(s) at %ux%u (scene '%s')\n", options.frames, options.width, options.height,
                options.scene.c_str());

    vfs::VirtualFileSystem vfs;
    vfs.mount(exeDir, "/", true);

    rhi::DeviceConfig deviceConfig;
    deviceConfig.headless = true;
    deviceConfig.enableValidation = options.validation;
    auto device = std::make_unique<rhi::Device>(deviceConfig);
    log->info("device: {} (headless)", device->deviceName());
    std::printf("[headless] device: %s\n", device->deviceName().c_str());

    render::Renderer renderer(*device, vfs, 1);

    auto cube = render::Mesh::makeCube(*device);
    auto ground = render::Mesh::makeGroundPlane(*device, 200.0f);

    int result = 0;

    if (!options.contactSheet.empty())
    {
        result = runContactSheet(*device, renderer, cube, ground, options.contactSheet);
    }
    else
    {
        Offscreen target;
        createTargets(*device, target, {options.width, options.height});
        renderer.ensureTargets(target.extent, kColorFormat);
        if (!renderer.ready())
        {
            log->critical("renderer is not ready (missing shaders?)");
            std::fprintf(stderr, "[headless] renderer not ready\n");
            device->waitIdle();
            return 1;
        }

        render::Scene scene;
        buildScene(options.scene, scene, cube, ground);

        std::unique_ptr<render::DebugDraw> debug;
        if (options.debugDraw)
        {
            debug = std::make_unique<render::DebugDraw>(*device, vfs);
            debug->ensureTargets(kColorFormat, target.extent);
            if (!debug->ready())
            {
                std::fprintf(stderr, "[headless] debug draw pipeline unavailable\n");
            }
        }

        const std::uint32_t frames = options.frames == 0 ? 1 : options.frames;
        const std::vector<std::uint8_t> pixels =
            renderFrames(*device, renderer, target, scene, frames, options.resizeFrame, true, debug.get());

        if (!options.screenshot.empty())
        {
            if (writePng(options.screenshot, pixels, target.extent))
            {
                std::printf("[headless] wrote %s (%ux%u)\n", options.screenshot.string().c_str(), target.extent.width, target.extent.height);
            }
            else
            {
                std::fprintf(stderr, "[headless] failed to write %s\n", options.screenshot.string().c_str());
                result = 1;
            }
        }

        if (options.semantic && result == 0)
        {
            if (!checkSemantics(scene, sceneProbes(options.scene), pixels, target.extent))
            {
                result = 4;
            }
        }

        if (options.verifyGolden || options.updateGolden)
        {
            const std::string goldenName = options.scene + (options.debugDraw ? "_debug" : "");
            std::filesystem::path reference = options.goldenDir / (goldenName + ".png");
            if (options.updateGolden)
            {
                std::filesystem::create_directories(options.goldenDir);
                if (writePng(reference, pixels, target.extent))
                {
                    std::printf("[headless] updated golden %s (%ux%u)\n", reference.string().c_str(), target.extent.width,
                                target.extent.height);
                }
                else
                {
                    std::fprintf(stderr, "[headless] failed to write golden %s\n", reference.string().c_str());
                    result = 1;
                }
            }
            else if (result == 0)
            {
                double diffPercent = 0.0;
                double psnr = 0.0;
                if (!compareGolden(reference, pixels, target.extent, options.maxDiffPercent, options.minPsnr, diffPercent, psnr))
                {
                    std::fprintf(stderr, "[headless] golden mismatch for scene '%s'\n", options.scene.c_str());
                    result = 3;
                }
            }
        }
    }

    device->waitIdle();

    const std::uint32_t messages = device->validationMessageCount();
    if (messages > 0)
    {
        std::fprintf(stderr, "[headless] %u Vulkan validation message(s) emitted\n", messages);
    }

    if (result != 0)
    {
        return result;
    }
    return messages > 0 ? 2 : 0;
}

} // namespace sim::app
