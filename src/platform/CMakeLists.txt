# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_platform STATIC
    src/Input.cpp
    src/Sdl.cpp
    src/Window.cpp)
add_library(sim::platform ALIAS sim_platform)

target_include_directories(sim_platform PUBLIC include)
target_link_libraries(sim_platform
    PUBLIC
        sim::core
        sim::input
        SDL3::SDL3
        volk::volk)
sim_configure_target(sim_platform)
