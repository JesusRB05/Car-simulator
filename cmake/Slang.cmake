# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Slang is not available in vcpkg, so the pinned prebuilt release is fetched
# with FetchContent (as allowed by the project brief). Only the `slangc`
# command-line compiler is needed at build time; shaders are compiled to
# SPIR-V and a reflection JSON is emitted next to each output so that pipeline
# layouts can be generated from Slang's reflection data.

include(FetchContent)

set(SIM_SLANGC "" CACHE FILEPATH "Path to the slangc executable (skips the FetchContent download)")

if(NOT SIM_SLANGC AND DEFINED ENV{SIM_SLANGC})
    set(SIM_SLANGC "$ENV{SIM_SLANGC}")
endif()

if(NOT SIM_SLANGC)
    set(_slang_version "2026.19")
    if(WIN32)
        set(_slang_slug "windows-x86_64.zip")
        set(_slang_bin "bin/slangc.exe")
    elseif(APPLE)
        set(_slang_slug "macos-x86_64.zip")
        set(_slang_bin "bin/slangc")
    else()
        set(_slang_slug "linux-x86_64.zip")
        set(_slang_bin "bin/slangc")
    endif()

    FetchContent_Declare(slang_prebuilt
        URL "https://github.com/shader-slang/slang/releases/download/v${_slang_version}/slang-${_slang_version}-${_slang_slug}"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

    FetchContent_MakeAvailable(slang_prebuilt)
    set(SIM_SLANGC "${slang_prebuilt_SOURCE_DIR}/${_slang_bin}")
endif()

if(NOT EXISTS "${SIM_SLANGC}")
    message(FATAL_ERROR "slangc not found at '${SIM_SLANGC}'. "
        "Set -DSIM_SLANGC=<path to slangc> or -DSIM_ENABLE_SLANG=OFF.")
endif()

message(STATUS "Using slangc: ${SIM_SLANGC}")

# sim_slang_shader(<target> <shader-file> <entry-point> <stage>)
#   Compiles one entry point of a .slang file to SPIR-V (and reflection JSON)
#   at build time and copies the artefacts next to the target executable.
#
#   The copy next to the executable is a custom command with its own OUTPUT,
#   not a POST_BUILD: a POST_BUILD only runs when the target relinks and ninja
#   treats shader dependencies as order-only, so a pure shader edit never
#   re-triggered the copy and the runtime kept serving stale SPIR-V.
function(sim_slang_shader target shader entry stage)
    get_filename_component(_name "${shader}" NAME_WE)
    set(_outdir "${CMAKE_BINARY_DIR}/shaders")
    file(MAKE_DIRECTORY "${_outdir}")
    set(_spv  "${_outdir}/${_name}.${stage}.spv")
    set(_json "${_outdir}/${_name}.${stage}.json")
    # The top-level project pins a single runtime output directory for all
    # binaries, so the copy edge can target the executable's directory at
    # configure time instead of via $<TARGET_FILE_DIR:...> (which cannot be
    # evaluated inside custom-command OUTPUT genexes).
    set(_bin_dir "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders")
    set(_bin_spv  "${_bin_dir}/${_name}.${stage}.spv")
    set(_bin_json "${_bin_dir}/${_name}.${stage}.json")

    add_custom_command(
        OUTPUT "${_spv}" "${_json}"
        COMMAND "${SIM_SLANGC}" "${shader}"
                -target spirv
                -emit-spirv-via-glsl
                -entry "${entry}"
                -stage "${stage}"
                -fvk-use-entrypoint-name
                -o "${_spv}"
                -reflection-json "${_json}"
        DEPENDS "${shader}"
        COMMENT "Slang: ${_name} [${stage}]"
        VERBATIM)

    add_custom_command(
        OUTPUT "${_bin_spv}" "${_bin_json}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${_bin_dir}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_spv}"  "${_bin_spv}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_json}" "${_bin_json}"
        DEPENDS "${_spv}" "${_json}"
        VERBATIM)

    add_custom_target(${target}_shader_${_name}_${stage} DEPENDS "${_bin_spv}" "${_bin_json}")
    add_dependencies(${target} ${target}_shader_${_name}_${stage})
endfunction()
