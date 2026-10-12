# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Optional integration of AMD FidelityFX Super Resolution (Vulkan backend).
# Nothing consumes it yet: this only fetches the pinned sources and exposes
# their headers through sim::fsr so the build graph is ready for a later step.
# Disabled by default because the fetch requires network access.

include(FetchContent)

FetchContent_Declare(fsr
    GIT_REPOSITORY https://github.com/GPUOpen-Effects/FidelityFX-FSR.git
    GIT_TAG        v1.0.3)

# The FSR repository is a source drop without a buildable top-level project.
FetchContent_MakeAvailable(fsr)

add_library(sim_fsr INTERFACE)
add_library(sim::fsr ALIAS sim_fsr)
target_include_directories(sim_fsr INTERFACE "${fsr_SOURCE_DIR}/Kits/FidelityFX")
message(STATUS "FSR sources fetched at ${fsr_SOURCE_DIR} (not linked yet)")
