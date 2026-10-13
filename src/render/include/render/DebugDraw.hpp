// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "render/Camera.hpp"
#include "render/ShaderCache.hpp"
#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"
#include "rhi/Pipeline.hpp"
#include "rhi/Resources.hpp"
#include "rhi/Types.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::render {

// A line instancing debug renderer. The CPU builds a list of coloured line
// segments; render() uploads them into one instance buffer and issues a single
// instanced line-list draw. It draws over the scene without depth testing, so
// it is meant as an overlay (grids, axes, collision shapes) rather than depth
// corrected geometry.
class DebugDraw {
public:
    DebugDraw(rhi::Device& device, vfs::VirtualFileSystem& vfs, std::uint32_t maxInstances = 16384);
    ~DebugDraw();

    DebugDraw(const DebugDraw&) = delete;
    DebugDraw& operator=(const DebugDraw&) = delete;

    bool ready() const { return pipeline_ != nullptr; }

    // (Re)creates the pipeline when the colour format or target size changes.
    void ensureTargets(rhi::Format colorFormat, rhi::Extent2D extent);

    void clear() { instances_.clear(); }
    bool empty() const { return instances_.empty(); }

    void addLine(const glm::vec3& start, const glm::vec3& end, const glm::vec4& color);
    void addAxes(const glm::vec3& origin, float length);
    void addGrid(float halfSize, float step, const glm::vec4& lineColor, const glm::vec4& axisColor);

    // Draws every queued segment over `colorAttachment` (which must already be
    // in the ColorAttachment layout). Clears the queue afterwards.
    void render(rhi::CommandList& commandList, const Camera& camera, const rhi::ColorAttachment& colorAttachment);

private:
    struct Instance {
        glm::vec3 start{0.0f};
        glm::vec3 end{0.0f};
        glm::vec4 color{1.0f};
    };

    void createPipeline(rhi::Format colorFormat, rhi::Extent2D extent);

    rhi::Device& device_;
    vfs::VirtualFileSystem& vfs_;
    ShaderCache shaders_;
    std::uint32_t maxInstances_ = 0;
    rhi::Format colorFormat_ = rhi::Format::Undefined;
    rhi::Extent2D extent_{};

    std::unique_ptr<rhi::DescriptorSetLayout> layout_;
    std::unique_ptr<rhi::DescriptorPool> pool_;
    rhi::DescriptorSet set_;
    std::unique_ptr<rhi::Buffer> uniformBuffer_;
    std::unique_ptr<rhi::Buffer> instanceBuffer_;
    std::unique_ptr<rhi::GraphicsPipeline> pipeline_;
    std::vector<Instance> instances_;
};

} // namespace sim::render
