// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <volk.h>

#include <cstdint>
#include <vector>

#include "rhi/CommandList.hpp"
#include "rhi/Device.hpp"

namespace sim::rhi {

// Measures GPU time of a command-buffer region with timestamp queries. One pair
// of queries is kept per frame in flight; the previous result for a slot is read
// back when that slot is reused (its fence has already signalled), so reading
// never stalls the pipeline.
class GpuTimer {
public:
    GpuTimer(Device& device, std::uint32_t slots);
    ~GpuTimer();

    GpuTimer(const GpuTimer&) = delete;
    GpuTimer& operator=(const GpuTimer&) = delete;

    // Reads the last completed measurement for the slot, or -1 if none yet.
    double readMilliseconds(std::uint32_t slot);

    // Clears the slot's queries and brackets the region of interest.
    void begin(CommandList& commandList, std::uint32_t slot);
    void end(CommandList& commandList, std::uint32_t slot);

private:
    Device* device_ = nullptr;
    VkQueryPool pool_ = VK_NULL_HANDLE;
    std::uint32_t slots_ = 0;
    float periodNs_ = 1.0f;
    std::vector<bool> written_;
};

} // namespace sim::rhi
