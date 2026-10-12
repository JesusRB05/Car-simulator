// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <SDL3/SDL.h>
#include <volk.h>

namespace sim::platform {

class InputManager;

struct WindowDesc
{
    std::string title = "Car Simulator";
    int width = 1280;
    int height = 720;
    bool resizable = true;
    bool hidden = false;
};

// SDL3 window. Owns the platform window and, on request, the Vulkan surface
// created from it. The RHI layer never sees SDL: the application asks the
// window for a surface and hands the raw handle to the RHI.
class Window
{
public:
    explicit Window(const WindowDesc& desc);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    void setInput(InputManager* input) { input_ = input; }
    void setEventHook(std::function<void(const SDL_Event&)> hook) { eventHook_ = std::move(hook); }

    // Drains the SDL event queue, feeding the input manager and event hook.
    // Returns false once a quit was requested.
    bool pumpEvents();

    bool shouldClose() const { return closeRequested_; }
    void requestClose() { closeRequested_ = true; }

    SDL_Window* handle() const { return window_; }

    void getDrawableSize(int& width, int& height) const;
    void getWindowSize(int& width, int& height) const;
    bool isMinimized() const;

    // True if the drawable size changed since the last clearResized().
    bool wasResized() const { return resized_; }
    void clearResized() { resized_ = false; }

    void setTitle(std::string_view title);

    VkSurfaceKHR createVulkanSurface(VkInstance instance) const;
    static std::vector<const char*> requiredVulkanInstanceExtensions();

private:
    SDL_Window* window_ = nullptr;
    InputManager* input_ = nullptr;
    std::function<void(const SDL_Event&)> eventHook_;
    bool closeRequested_ = false;
    bool resized_ = true;
    bool minimized_ = false;
};

} // namespace sim::platform
