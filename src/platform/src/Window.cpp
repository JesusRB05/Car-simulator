// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "platform/Window.hpp"

#include <SDL3/SDL_vulkan.h>

#include "core/Log.hpp"
#include "platform/Input.hpp"

namespace sim::platform {

Window::Window(const WindowDesc& desc) {
    SDL_WindowFlags flags = SDL_WINDOW_VULKAN;
    if (desc.resizable) {
        flags |= SDL_WINDOW_RESIZABLE;
    }
    if (desc.hidden) {
        flags |= SDL_WINDOW_HIDDEN;
    }

    window_ = SDL_CreateWindow(desc.title.c_str(), desc.width, desc.height, flags);
    if (window_ == nullptr) {
        core::Log::get("platform")->critical("SDL_CreateWindow failed: {}", SDL_GetError());
        return;
    }
    core::Log::get("platform")->info("created window '{}' ({}x{})", desc.title, desc.width, desc.height);
}

Window::~Window() {
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
}

bool Window::pumpEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (eventHook_) {
            eventHook_(event);
        }
        if (input_ != nullptr) {
            input_->processEvent(event);
        }

        switch (event.type) {
        case SDL_EVENT_QUIT:
            closeRequested_ = true;
            break;
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            if (window_ != nullptr && event.window.windowID == SDL_GetWindowID(window_)) {
                closeRequested_ = true;
            }
            break;
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
        case SDL_EVENT_WINDOW_RESIZED:
            resized_ = true;
            break;
        case SDL_EVENT_WINDOW_MINIMIZED:
            minimized_ = true;
            resized_ = true;
            break;
        case SDL_EVENT_WINDOW_RESTORED:
            minimized_ = false;
            resized_ = true;
            break;
        default:
            break;
        }
    }
    return !closeRequested_;
}

void Window::getDrawableSize(int& width, int& height) const {
    width = 0;
    height = 0;
    if (window_ != nullptr) {
        SDL_GetWindowSizeInPixels(window_, &width, &height);
    }
}

void Window::getWindowSize(int& width, int& height) const {
    width = 0;
    height = 0;
    if (window_ != nullptr) {
        SDL_GetWindowSize(window_, &width, &height);
    }
}

bool Window::isMinimized() const {
    if (window_ == nullptr) {
        return true;
    }
    return minimized_ || (SDL_GetWindowFlags(window_) & SDL_WINDOW_MINIMIZED) != 0;
}

void Window::setTitle(std::string_view title) {
    if (window_ != nullptr) {
        SDL_SetWindowTitle(window_, std::string(title).c_str());
    }
}

VkSurfaceKHR Window::createVulkanSurface(VkInstance instance) const {
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    if (window_ == nullptr) {
        return surface;
    }
    if (!SDL_Vulkan_CreateSurface(window_, instance, nullptr, &surface)) {
        core::Log::get("platform")->error("SDL_Vulkan_CreateSurface failed: {}", SDL_GetError());
        return VK_NULL_HANDLE;
    }
    return surface;
}

std::vector<const char*> Window::requiredVulkanInstanceExtensions() {
    std::vector<const char*> extensions;
    Uint32 count = 0;
    const char* const* names = SDL_Vulkan_GetInstanceExtensions(&count);
    if (names == nullptr) {
        core::Log::get("platform")->error("SDL_Vulkan_GetInstanceExtensions failed: {}", SDL_GetError());
        return extensions;
    }
    extensions.reserve(count);
    for (Uint32 i = 0; i < count; ++i) {
        extensions.push_back(names[i]);
    }
    return extensions;
}

} // namespace sim::platform
