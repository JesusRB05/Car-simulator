// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "rhi/GpuTimer.hpp"

#include <stdexcept>

namespace sim::rhi {

GpuTimer::GpuTimer(Device& device, std::uint32_t slots) : device_(&device), slots_(slots < 1 ? 1 : slots) {
    VkQueryPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    info.queryType = VK_QUERY_TYPE_TIMESTAMP;
    info.queryCount = slots_ * 2;

    if (vkCreateQueryPool(device.handle(), &info, nullptr, &pool_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create a GPU timestamp query pool");
    }

    periodNs_ = device.properties().limits.timestampPeriod;
    if (periodNs_ <= 0.0f) {
        periodNs_ = 1.0f;
    }
    written_.assign(slots_, false);
}

GpuTimer::~GpuTimer() {
    if (pool_ != VK_NULL_HANDLE) {
        vkDestroyQueryPool(device_->handle(), pool_, nullptr);
    }
}

double GpuTimer::readMilliseconds(std::uint32_t slot) {
    if (slot >= slots_ || !written_[slot]) {
        return -1.0;
    }

    std::uint64_t stamps[2] = {0, 0};
    const VkResult result =
        vkGetQueryPoolResults(device_->handle(), pool_, slot * 2, 2, sizeof(stamps), stamps, sizeof(std::uint64_t), 0);
    if (result != VK_SUCCESS) {
        return -1.0;
    }

    const std::uint64_t delta = stamps[1] >= stamps[0] ? stamps[1] - stamps[0] : 0;
    return static_cast<double>(delta) * static_cast<double>(periodNs_) * 1.0e-6;
}

void GpuTimer::begin(CommandList& commandList, std::uint32_t slot) {
    if (slot >= slots_) {
        return;
    }

    vkCmdResetQueryPool(commandList.handle(), pool_, slot * 2, 2);
    vkCmdWriteTimestamp(commandList.handle(), VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, pool_, slot * 2);
    written_[slot] = true;
}

void GpuTimer::end(CommandList& commandList, std::uint32_t slot) {
    if (slot >= slots_) {
        return;
    }

    vkCmdWriteTimestamp(commandList.handle(), VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, pool_, slot * 2 + 1);
}

} // namespace sim::rhi
