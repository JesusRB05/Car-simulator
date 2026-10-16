# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_render STATIC
    src/Camera.cpp
    src/Frustum.cpp
    src/Mesh.cpp
    src/Scene.cpp
    src/ShaderCache.cpp
    src/RenderGraph.cpp
    src/DebugDraw.cpp
    src/Renderer.cpp)
add_library(sim::render ALIAS sim_render)

target_include_directories(sim_render PUBLIC include)
target_link_libraries(sim_render
    PUBLIC
        sim::core
        sim::vfs
        sim::rhi
        glm::glm)
sim_configure_target(sim_render)
