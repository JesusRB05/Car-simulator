# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_asset STATIC
    src/Gltf.cpp
    src/StbImage.cpp)
add_library(sim::asset ALIAS sim_asset)

target_include_directories(sim_asset PUBLIC include)
target_link_libraries(sim_asset
    PUBLIC
        sim::core
        sim::vfs
        sim::cgltf
        sim::stb
        glm::glm)
sim_configure_target(sim_asset)
