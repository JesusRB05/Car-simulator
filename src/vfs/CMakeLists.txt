# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_vfs STATIC
    src/VirtualFileSystem.cpp)
add_library(sim::vfs ALIAS sim_vfs)

target_include_directories(sim_vfs PUBLIC include)
target_link_libraries(sim_vfs
    PUBLIC
        sim::core
        PhysFS::PhysFS)
sim_configure_target(sim_vfs)
