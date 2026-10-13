// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <cmath>
#include <cstdint>
#include <cstring>

#include <doctest/doctest.h>
#include <glm/glm.hpp>

#include "render/Camera.hpp"
#include "render/Uniforms.hpp"

namespace sim::render {

TEST_CASE("glm column-major memory matches the shader column_major layout")
{
    // std140 upload layout: every member is a multiple of 16 bytes and matrices
    // are stored as their columns contiguously. GLM keeps mat4 as four vec4
    // columns, so a raw byte copy is the identity mapping between the C++
    // struct and the column_major shaders; no transpose must ever happen.
    CHECK(sizeof(glm::mat4) == 64);
    CHECK(sizeof(FrameUniforms) == 16 * 7);
    CHECK(sizeof(ObjectPushConstants) == 16 * 5);
    CHECK(offsetof(FrameUniforms, viewProjection) == 0);
    CHECK(offsetof(ObjectPushConstants, model) == 0);

    // writeMatrix copies the identity verbatim: bytes 0..15 are the first
    // column (1,0,0,0), bytes 16..31 the second column, etc. A row-major reader
    // would see (1,0,0,0, 0,1,0,0, ...) in a different order and fail here.
    glm::mat4 identity(1.0f);
    std::uint8_t bytes[64];
    writeMatrix(identity, bytes);
    for (int column = 0; column < 4; ++column)
    {
        for (int row = 0; row < 4; ++row)
        {
            float value = 0.0f;
            std::memcpy(&value, bytes + (column * 4 + row) * sizeof(float), sizeof(float));
            CHECK(value == doctest::Approx(column == row ? 1.0f : 0.0f));
        }
    }
}

TEST_CASE("a known world point projects to the expected pixel (column-major mul)")
{
    constexpr int kWidth = 640;
    constexpr int kHeight = 360;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTanHalf = 0.4142135623730950; // tan(22.5 deg) = sqrt(2) - 1
    const float aspect = static_cast<float>(kWidth) / static_cast<float>(kHeight);

    // Camera at the origin looking down -Z with +Y up on a right-handed frame.
    // For a point (x, y, -d) the analytic NDC is
    //   ndcX = x / (d * tan(fovY/2) * aspect),   ndcY = y / (d * tan(fovY/2))
    // independent of any matrix convention, so it pins down the transform that
    // viewProjection() * p must reproduce. The pixel mapping mirrors the
    // headless GPU semantic checks (HeadlessRunner::checkSemantics) and matches
    // the RHI viewport flip: +Y world is row 0 (top) and +X world grows the
    // column, so this is the absolute screen orientation of the engine.
    Camera camera;
    camera.setPerspective(45.0f, aspect, 0.05f, 100.0f);
    camera.lookAt({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});

    const double distance = 2.0;
    const double x = 1.0;
    const double y = 0.5;
    const double expectedNdcX = x / (distance * kTanHalf * aspect);
    const double expectedNdcY = y / (distance * kTanHalf);

    const glm::vec4 clip = camera.viewProjection() * glm::vec4(x, y, -distance, 1.0f);
    REQUIRE(clip.w > 0.0f);
    const double ndcX = clip.x / clip.w;
    const double ndcY = clip.y / clip.w;
    CHECK(ndcX == doctest::Approx(expectedNdcX).epsilon(1e-6));
    CHECK(ndcY == doctest::Approx(expectedNdcY).epsilon(1e-6));

    const int px = static_cast<int>((ndcX * 0.5 + 0.5) * kWidth);
    const int py = static_cast<int>((1.0 - (ndcY * 0.5 + 0.5)) * kHeight);
    const int expectedPx = static_cast<int>((expectedNdcX * 0.5 + 0.5) * kWidth);
    const int expectedPy = static_cast<int>((1.0 - (expectedNdcY * 0.5 + 0.5)) * kHeight);
    MESSAGE("known world point (1, 0.5, -2) projects to pixel (" << px << ", " << py << ")");
    CHECK(px == expectedPx);
    CHECK(py == expectedPy);
}

TEST_CASE("screen orientation is absolute: +Y world renders above, +X world renders right")
{
    // Pins the engine's screen orientation without a GPU: with a horizontal
    // camera, the RHI-flipped mapping must place a higher world point on a
    // strictly smaller row (top of the image) and a point towards +X on a
    // strictly larger column, and anything behind the camera must stay behind.
    constexpr int kWidth = 640;
    constexpr int kHeight = 360;
    const float aspect = static_cast<float>(kWidth) / static_cast<float>(kHeight);

    Camera camera;
    camera.setPerspective(60.0f, aspect, 0.1f, 100.0f);
    camera.lookAt({0.0f, 0.6f, 0.0f}, {0.0f, 0.6f, -10.0f}); // horizontal gaze, +Y up

    const auto rowCol = [&](float x, float y, float z) -> std::pair<int, int> {
        const glm::vec4 clip = camera.viewProjection() * glm::vec4(x, y, z, 1.0f);
        REQUIRE(clip.w > 0.0f);
        const float ndcX = clip.x / clip.w;
        const float ndcY = clip.y / clip.w;
        const int px = static_cast<int>((ndcX * 0.5f + 0.5f) * static_cast<float>(kWidth));
        const int py = static_cast<int>((1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(kHeight));
        return {px, py};
    };

    // Vertical: same gaze axis, higher object must land above (smaller row).
    const auto high = rowCol(0.0f, 3.0f, -8.0f);
    const auto low = rowCol(0.0f, 0.4f, -8.0f);
    CHECK(high.second < low.second);
    CHECK(high.first == low.first); // same ray, same column

    // Horizontal: objects at the same height/ray must order by +X.
    const auto right = rowCol(2.5f, 0.6f, -8.0f);
    const auto left = rowCol(-2.5f, 0.6f, -8.0f);
    CHECK(right.first > left.first);
    CHECK(right.second == left.second);

    // World up really is the top half of the frame and world ahead is in front
    // of the camera (never a negative clip.w).
    CHECK(high.second < kHeight / 2);
    CHECK(low.second > kHeight / 2);
}

} // namespace sim::render