// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "render/ShaderCache.hpp"

#include <cstddef>
#include <stdexcept>

#include "core/Log.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::render {

ShaderCache::ShaderCache(rhi::Device& device, vfs::VirtualFileSystem& vfs) : device_(device), vfs_(vfs) {}

const rhi::ShaderModule* ShaderCache::get(std::string_view virtualPath) {
    const std::string path(virtualPath);
    const auto existing = modules_.find(path);
    if (existing != modules_.end()) {
        return existing->second.get();
    }

    const auto bytes = vfs_.readBinary(path);
    if (!bytes || bytes->empty()) {
        core::Log::get("render")->error("shader '{}' is missing from the VFS", path);
        return nullptr;
    }

    auto module =
        std::make_unique<rhi::ShaderModule>(device_, reinterpret_cast<const std::byte*>(bytes->data()), bytes->size());
    const rhi::ShaderModule* result = module.get();
    modules_.emplace(path, std::move(module));
    return result;
}

} // namespace sim::render
