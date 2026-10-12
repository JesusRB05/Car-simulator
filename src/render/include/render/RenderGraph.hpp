// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "rhi/CommandList.hpp"

namespace sim::render {

// A minimal, linear render graph. Each pass records work into the frame's
// command list in registration order. It exists so passes can be added and
// reordered without touching the frame loop.
class RenderGraph
{
public:
    using PassFunction = std::function<void(rhi::CommandList&)>;

    void addPass(std::string name, PassFunction pass);
    void clear() { passes_.clear(); }
    bool empty() const { return passes_.empty(); }

    void execute(rhi::CommandList& commandList);

    const std::vector<std::pair<std::string, PassFunction>>& passes() const { return passes_; }

private:
    std::vector<std::pair<std::string, PassFunction>> passes_;
};

} // namespace sim::render
