# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Shared helpers for configuring the project's own targets.

# sim_configure_target(<target>)
#   Applies the C++ standard, warning set and common definitions to a
#   first-party target. Third-party targets are never passed through this.
function(sim_configure_target target)
    target_compile_features(${target} PUBLIC cxx_std_20)
    set_target_properties(${target} PROPERTIES
        CXX_EXTENSIONS OFF
        POSITION_INDEPENDENT_CODE ON)

    if(SIM_WARNINGS)
        if(MSVC)
            target_compile_options(${target} PRIVATE /W4 /permissive- /Zc:__cplusplus /Zc:preprocessor /utf-8)
        else()
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
        endif()
    endif()

    if(SIM_WARNINGS_AS_ERRORS)
        if(MSVC)
            target_compile_options(${target} PRIVATE /WX)
        else()
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()

    target_compile_definitions(${target} PRIVATE
        $<$<CONFIG:Debug>:SIM_DEBUG=1>
        $<$<CONFIG:Release>:SIM_RELEASE=1>)
endfunction()

# sim_enable_deterministic_fp(<target>)
#   Strict, reproducible floating point for physics modules. Must never be
#   combined with -ffast-math or any fast-math equivalent.
function(sim_enable_deterministic_fp target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /fp:precise)
    else()
        target_compile_options(${target} PRIVATE -ffp-contract=off)
    endif()
    target_compile_definitions(${target} PRIVATE SIM_DETERMINISTIC_FP=1)
endfunction()
