// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/RenderGraph.hpp"

namespace sim::render {

void RenderGraph::addPass(std::string name, PassFunction pass)
{
    passes_.emplace_back(std::move(name), std::move(pass));
}

void RenderGraph::execute(rhi::CommandList& commandList)
{
    for (auto& [name, pass] : passes_)
    {
        (void)name;
        if (pass)
        {
            pass(commandList);
        }
    }
}

} // namespace sim::render
