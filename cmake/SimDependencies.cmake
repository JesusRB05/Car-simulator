# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Locates every vcpkg dependency and exposes small, uniform alias targets so
# module CMakeLists never have to care about upstream target naming quirks.

# volk may try to pull in a full Vulkan SDK; we only want the headers it ships via vcpkg.
set(VOLK_PULL_IN_VULKAN OFF CACHE BOOL "" FORCE)

find_package(glm CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)
find_package(SDL3 CONFIG REQUIRED)
find_package(PhysFS CONFIG REQUIRED)
find_package(volk CONFIG REQUIRED)
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
find_package(ktx CONFIG REQUIRED)
find_package(meshoptimizer CONFIG REQUIRED)
find_package(sol2 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(doctest CONFIG REQUIRED)
find_package(Tracy CONFIG REQUIRED)

if(SIM_ENABLE_OPENXR)
    find_package(OpenXR CONFIG REQUIRED)
endif()

# ---------------------------------------------------------------------------
# LuaJIT ships no CMake package config through vcpkg; describe it by hand.
# ---------------------------------------------------------------------------
find_path(SIM_LUAJIT_INCLUDE_DIR NAMES lua.h PATH_SUFFIXES luajit REQUIRED)
find_library(SIM_LUAJIT_LIBRARY NAMES lua51 lua5.1 luajit REQUIRED)
add_library(sim_luajit INTERFACE)
add_library(sim::luajit ALIAS sim_luajit)
target_include_directories(sim_luajit INTERFACE "${SIM_LUAJIT_INCLUDE_DIR}")
target_link_libraries(sim_luajit INTERFACE "${SIM_LUAJIT_LIBRARY}")

# ---------------------------------------------------------------------------
# Header-only dependencies without a package config.
# ---------------------------------------------------------------------------
find_path(SIM_STB_INCLUDE_DIR       NAMES stb_image.h REQUIRED)
find_path(SIM_CGLTF_INCLUDE_DIR     NAMES cgltf.h    REQUIRED)
find_path(SIM_MINIAUDIO_INCLUDE_DIR NAMES miniaudio.h REQUIRED)

add_library(sim_stb INTERFACE)
add_library(sim::stb ALIAS sim_stb)
target_include_directories(sim_stb INTERFACE "${SIM_STB_INCLUDE_DIR}")

add_library(sim_cgltf INTERFACE)
add_library(sim::cgltf ALIAS sim_cgltf)
target_include_directories(sim_cgltf INTERFACE "${SIM_CGLTF_INCLUDE_DIR}")

add_library(sim_miniaudio INTERFACE)
add_library(sim::miniaudio ALIAS sim_miniaudio)
target_include_directories(sim_miniaudio INTERFACE "${SIM_MINIAUDIO_INCLUDE_DIR}")

# ---------------------------------------------------------------------------
# Tracy is optional: expose it as sim::profiling, empty when disabled.
# ---------------------------------------------------------------------------
add_library(sim_profiling INTERFACE)
add_library(sim::profiling ALIAS sim_profiling)
if(SIM_ENABLE_PROFILING)
    target_link_libraries(sim_profiling INTERFACE Tracy::TracyClient)
    target_compile_definitions(sim_profiling INTERFACE SIM_PROFILING=1)
endif()

# ---------------------------------------------------------------------------
# OpenXR is integrated but not used yet.
# ---------------------------------------------------------------------------
add_library(sim_openxr INTERFACE)
add_library(sim::openxr ALIAS sim_openxr)
if(SIM_ENABLE_OPENXR)
    target_link_libraries(sim_openxr INTERFACE OpenXR::openxr_loader)
endif()
