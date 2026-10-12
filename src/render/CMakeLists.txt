# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_render STATIC
    src/Camera.cpp
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

if(SIM_ENABLE_SLANG)
    sim_slang_shader(sim_render "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh.slang" vertexMain vertex)
    sim_slang_shader(sim_render "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh.slang" fragmentMain fragment)
    sim_slang_shader(sim_render "${CMAKE_CURRENT_SOURCE_DIR}/shaders/debug.slang" vertexMain vertex)
    sim_slang_shader(sim_render "${CMAKE_CURRENT_SOURCE_DIR}/shaders/debug.slang" fragmentMain fragment)
endif()
