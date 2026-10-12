# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# ImGui core comes from vcpkg. Its backends and ImPlot however are compiled here
# from pinned upstream sources: the Vulkan backend needs
# IMGUI_IMPL_VULKAN_USE_VOLK (volk defines the Vulkan entry points as global
# function pointers, which clashes with the loader-based prototypes the vcpkg
# backends would otherwise reference), and the vcpkg ImPlot ships an imgui.h
# that would otherwise collide with the headers the backends are built against.

include(FetchContent)

FetchContent_Declare(imgui_sources
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.9b
    GIT_SHALLOW    TRUE)

# Upstream imgui has no top-level CMakeLists.txt, so MakeAvailable only
# populates the sources without configuring a project.
FetchContent_MakeAvailable(imgui_sources)

FetchContent_Declare(implot_sources
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        851cdbdd60d1dfb64c23e987969694de0f547a6b
    GIT_SHALLOW    TRUE)

# ImPlot has no CMake project either; compile it against the same imgui sources
# the backends use so every translation unit sees one consistent imgui.h.
FetchContent_MakeAvailable(implot_sources)

add_library(sim_imgui_backends STATIC
    "${imgui_sources_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp"
    "${imgui_sources_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp")

add_library(sim::imgui_backends ALIAS sim_imgui_backends)

target_include_directories(sim_imgui_backends PUBLIC
    "${imgui_sources_SOURCE_DIR}"
    "${imgui_sources_SOURCE_DIR}/backends")

target_compile_definitions(sim_imgui_backends PRIVATE IMGUI_IMPL_VULKAN_USE_VOLK)
target_link_libraries(sim_imgui_backends PUBLIC
    imgui::imgui
    SDL3::SDL3
    volk::volk)

sim_configure_target(sim_imgui_backends)
set_target_properties(sim_imgui_backends PROPERTIES CXX_STANDARD 20)

add_library(sim_implot STATIC
    "${implot_sources_SOURCE_DIR}/implot.cpp"
    "${implot_sources_SOURCE_DIR}/implot_items.cpp")

add_library(sim::implot ALIAS sim_implot)

target_include_directories(sim_implot PUBLIC
    "${imgui_sources_SOURCE_DIR}"
    "${implot_sources_SOURCE_DIR}")

target_link_libraries(sim_implot PUBLIC imgui::imgui)

sim_configure_target(sim_implot)
set_target_properties(sim_implot PROPERTIES CXX_STANDARD 20)

