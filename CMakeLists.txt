# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
cmake_minimum_required(VERSION 3.25)

project(CarSimulator
    VERSION 0.1.0
    DESCRIPTION "Open-source, community driving simulator with a custom physics engine and a custom Vulkan renderer"
    HOMEPAGE_URL "https://github.com/car-simulator/car-simulator"
    LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# Ninja records header dependencies by matching cl's /showIncludes prefix byte
# for byte. A localized MSVC prints a non-ASCII prefix (Spanish: "Nota:
# inclusión del archivo:") in the current console code page, while CMake writes
# it in the OEM code page. Any mismatch means no header dependency is recorded
# and editing a header leaves stale objects with an old struct layout. The
# console code page is not stable during a build: `cmake --build` and parallel
# cl processes compiling with /utf-8 switch it, so roughly half of the objects
# lose their dependencies. The Windows presets set VSLANG=1033, which makes the
# prefix plain ASCII once the English language pack is installed.
if(MSVC AND CMAKE_GENERATOR MATCHES "Ninja" AND CMAKE_CL_SHOWINCLUDES_PREFIX MATCHES "[^ -~]")
    message(WARNING
        "cl prints a localized /showIncludes prefix ('${CMAKE_CL_SHOWINCLUDES_PREFIX}'), so Ninja "
        "loses header dependencies and header edits can leave stale objects. Install the English "
        "language pack for Visual Studio (Visual Studio Installer > Modify > Language packs) so that "
        "VSLANG=1033 takes effect. Until then, rebuild from clean after editing a header.")
endif()

# All binaries land in a single directory so runtime assets (shaders, DLLs, content) are easy to find.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
foreach(_cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_cfg} "${CMAKE_BINARY_DIR}/bin")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_cfg} "${CMAKE_BINARY_DIR}/bin")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_cfg} "${CMAKE_BINARY_DIR}/lib")
endforeach()

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(SIM_BUILD_TESTS       "Build the doctest unit-test suite"        ON)
option(SIM_WARNINGS          "Enable a strict warning set"              ON)
option(SIM_WARNINGS_AS_ERRORS "Treat warnings as errors"                OFF)
option(SIM_ENABLE_PROFILING  "Link and enable the Tracy profiler"       OFF)
option(SIM_ENABLE_SLANG      "Compile .slang shaders to SPIR-V"         ON)
option(SIM_ENABLE_OPENXR     "Integrate the OpenXR loader"              ON)
option(SIM_ENABLE_FSR        "Fetch AMD FidelityFX FSR sources (unused)" OFF)

include(SimHelpers)
include(SimDependencies)
include(ImGuiBackends)
if(SIM_ENABLE_SLANG)
    include(Slang)
endif()
if(SIM_ENABLE_FSR)
    include(Fsr)
endif()

add_subdirectory(src)
add_subdirectory(apps/sim)
add_subdirectory(plugins/example_native)

if(SIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

message(STATUS "")
message(STATUS "Car Simulator ${PROJECT_VERSION}")
message(STATUS "  build type .......... ${CMAKE_BUILD_TYPE}")
message(STATUS "  tests ............... ${SIM_BUILD_TESTS}")
message(STATUS "  profiling (Tracy) ... ${SIM_ENABLE_PROFILING}")
message(STATUS "  slang shaders ....... ${SIM_ENABLE_SLANG}")
message(STATUS "  OpenXR .............. ${SIM_ENABLE_OPENXR}")
message(STATUS "  FSR sources ......... ${SIM_ENABLE_FSR}")
message(STATUS "  runtime output ...... ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "")
