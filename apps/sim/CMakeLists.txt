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
# shaders must live next to the executable (the shader custom targets only
# stage them next to the render library).
if(SIM_ENABLE_SLANG)
    add_custom_command(TARGET sim POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:sim>/shaders"
        COMMAND "${CMAKE_COMMAND}" -E copy_directory "${CMAKE_BINARY_DIR}/shaders" "$<TARGET_FILE_DIR:sim>/shaders"
        VERBATIM)
endif()

sim_configure_target(sim)
