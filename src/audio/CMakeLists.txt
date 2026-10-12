# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_audio STATIC
    src/AudioEngine.cpp)
add_library(sim::audio ALIAS sim_audio)

target_include_directories(sim_audio PUBLIC include)
target_link_libraries(sim_audio
    PUBLIC
        sim::core
        sim::vfs
        sim::miniaudio)
sim_configure_target(sim_audio)
