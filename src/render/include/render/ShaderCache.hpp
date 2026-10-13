// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "rhi/Device.hpp"
#include "rhi/Resources.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::render {

// Loads SPIR-V shader modules from the virtual file system and caches them by
// path. Returns nullptr (and logs) when a shader is missing or invalid.
class ShaderCache {
public:
    ShaderCache(rhi::Device& device, vfs::VirtualFileSystem& vfs);

    const rhi::ShaderModule* get(std::string_view virtualPath);

private:
    rhi::Device& device_;
    vfs::VirtualFileSystem& vfs_;
    std::unordered_map<std::string, std::unique_ptr<rhi::ShaderModule>> modules_;
};

} // namespace sim::render
