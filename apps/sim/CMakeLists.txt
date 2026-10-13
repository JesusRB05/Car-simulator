# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_executable(sim
    src/main.cpp
    src/Application.cpp
    src/ImGuiLayer.cpp
    src/HeadlessRunner.cpp
    src/PhysicsScenes.cpp
    src/Scenes.cpp)

find_package(Stb REQUIRED)
target_include_directories(sim PRIVATE src ${Stb_INCLUDE_DIR})

target_link_libraries(sim
    PRIVATE
        sim::core
        sim::platform
        sim::vfs
        sim::modding
        sim::scripting
        sim::plugin_api
        sim::rhi
        sim::render
        sim::physics
        sim::vehicle
        sim::telemetry
        sim::audio
        sim::net
        sim::imgui_backends
        imgui::imgui
        sim::implot
        volk::volk
        glm::glm
        SDL3::SDL3
        spdlog::spdlog)

target_compile_definitions(sim PRIVATE IMGUI_IMPL_VULKAN_USE_VOLK)

# The simulator mounts its own directory as the VFS root, so the compiled
# shaders must live next to it. Compilation outputs track build/shaders and the
# copy to bin/shaders is its own OUTPUT edge (a POST_BUILD would only run when
# the executable relinks, so pure shader edits would keep serving stale SPIR-V).
if(SIM_ENABLE_SLANG)
    set(_sim_shader_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../src/render/shaders")
    sim_slang_shader(sim "${_sim_shader_dir}/mesh.slang" vertexMain vertex)
    sim_slang_shader(sim "${_sim_shader_dir}/mesh.slang" fragmentMain fragment)
    sim_slang_shader(sim "${_sim_shader_dir}/debug.slang" vertexMain vertex)
    sim_slang_shader(sim "${_sim_shader_dir}/debug.slang" fragmentMain fragment)
endif()

sim_configure_target(sim)
