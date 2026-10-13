# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_executable(sim_tests
    test_main.cpp
    test_core.cpp
    test_modding.cpp
    test_physics.cpp
    test_render_math.cpp)

target_include_directories(sim_tests PRIVATE .)

target_link_libraries(sim_tests
    PRIVATE
        sim::core
        sim::modding
        sim::physics
        sim::render
        doctest::doctest)

sim_configure_target(sim_tests)

add_test(NAME sim_tests COMMAND sim_tests)

# Headless offscreen GPU smoke tests. They need a Vulkan ICD (SwiftShader
# locally, Mesa lavapipe in CI) supplied through VK_ICD_FILENAMES. Enable with
# -DSIM_ENABLE_GPU_TESTS=ON and run with `ctest -L gpu`.
option(SIM_ENABLE_GPU_TESTS "Register headless offscreen GPU smoke tests" OFF)
set(SIM_VULKAN_ICD "" CACHE FILEPATH "Vulkan ICD JSON used by the headless GPU tests")

if(SIM_ENABLE_GPU_TESTS)
    set(_shot "${CMAKE_BINARY_DIR}/screenshots")
    file(MAKE_DIRECTORY "${_shot}")

    add_test(NAME sim_headless_default
             COMMAND $<TARGET_FILE:sim> --headless --frames 2 --screenshot "${_shot}/default.png" --scene default)
    add_test(NAME sim_headless_resize
             COMMAND $<TARGET_FILE:sim> --headless --frames 4 --resize 2 --screenshot "${_shot}/resize.png" --scene default)
    add_test(NAME sim_headless_validation
             COMMAND $<TARGET_FILE:sim> --headless --frames 2 --validation --screenshot "${_shot}/validation.png" --scene default)

    # Contact sheet: one labelled mosaic of every scene, for human review.
    add_test(NAME sim_contact_sheet
             COMMAND $<TARGET_FILE:sim> --headless --contact-sheet "${_shot}/contact_sheet.png")

    # Golden-image regression + semantic (projected pixel) checks per scene.
    # References live in tests/golden/<driver>/, keyed by the Vulkan driver that
    # produced them (lavapipe in CI, SwiftShader as an alternative). Picking a
    # different driver transparently switches the whole set.
    set(SIM_GOLDEN_DRIVER "lavapipe" CACHE STRING "Vulkan driver whose golden references are used")
    set(_golden "${CMAKE_SOURCE_DIR}/tests/golden/${SIM_GOLDEN_DRIVER}")
    set(_scenes default grid pyramid ramp orbit tower checker rows)
    set(_golden_tests "")
    foreach(_scene IN LISTS _scenes)
        set(_test "sim_golden_${_scene}")
        add_test(NAME ${_test}
                 COMMAND $<TARGET_FILE:sim> --headless --frames 2 --width 640 --height 360
                         --scene ${_scene} --verify-golden --golden-dir "${_golden}" --semantic)
        list(APPEND _golden_tests ${_test})
    endforeach()

    # The instanced debug-draw overlay (ground grid + world axes on the `grid`
    # scene) has its own reference so a regression in the instancing pipeline is
    # caught. No semantic probes: the overlay is decorative.
    add_test(NAME sim_golden_debug_overlay
             COMMAND $<TARGET_FILE:sim> --headless --frames 2 --width 640 --height 360
                     --scene grid --debug-draw --verify-golden --golden-dir "${_golden}")
    list(APPEND _golden_tests sim_golden_debug_overlay)

    set(_gpu_tests sim_headless_default sim_headless_resize sim_headless_validation sim_contact_sheet ${_golden_tests})
    set_tests_properties(${_gpu_tests} PROPERTIES LABELS gpu WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

    # Regenerate every golden reference for the selected driver (run manually
    # after intentional changes):
    #   cmake --build <build> --target update_golden
    add_custom_target(update_golden
        COMMAND ${CMAKE_COMMAND} -E echo "Regenerating golden images into ${_golden}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        VERBATIM)
    foreach(_scene IN LISTS _scenes)
        add_custom_command(TARGET update_golden POST_BUILD
            COMMAND $<TARGET_FILE:sim> --headless --frames 2 --width 640 --height 360
                    --scene ${_scene} --update-golden --golden-dir "${_golden}" --semantic
            VERBATIM)
    endforeach()
    add_custom_command(TARGET update_golden POST_BUILD
        COMMAND $<TARGET_FILE:sim> --headless --frames 2 --width 640 --height 360
                --scene grid --debug-draw --update-golden --golden-dir "${_golden}"
        VERBATIM)
    if(SIM_VULKAN_ICD)
        # The Vulkan loader on Windows rejects forward slashes in the ICD path.
        file(TO_NATIVE_PATH "${SIM_VULKAN_ICD}" _icd_native)
        set_tests_properties(${_gpu_tests} PROPERTIES ENVIRONMENT "VK_ICD_FILENAMES=${_icd_native}")
    endif()
endif()
