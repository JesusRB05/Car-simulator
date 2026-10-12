# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(example_native MODULE example_plugin.c)
target_include_directories(example_native PRIVATE ${PROJECT_SOURCE_DIR}/src/plugin_api/include)
set_target_properties(example_native PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/plugins"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/plugins")
sim_configure_target(example_native)
