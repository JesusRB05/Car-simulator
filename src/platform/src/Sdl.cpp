// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "platform/Sdl.hpp"

#include <SDL3/SDL.h>

#include "core/Log.hpp"

namespace sim::platform {

SdlContext::SdlContext() {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
        core::Log::get("platform")->critical("SDL_Init failed: {}", SDL_GetError());
        return;
    }
    if (!SDL_InitSubSystem(SDL_INIT_GAMEPAD)) {
        core::Log::get("platform")->warn("gamepad subsystem unavailable: {}", SDL_GetError());
    }
    if (!SDL_InitSubSystem(SDL_INIT_JOYSTICK)) {
        core::Log::get("platform")->warn("joystick subsystem unavailable: {}", SDL_GetError());
    }

    core::Log::get("platform")->info("SDL3 initialised (revision '{}')", SDL_GetRevision());
    valid_ = true;
}

SdlContext::~SdlContext() {
    if (valid_) {
        SDL_Quit();
        valid_ = false;
    }
}

} // namespace sim::platform
