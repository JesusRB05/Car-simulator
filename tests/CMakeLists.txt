# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_executable(sim_tests
    test_main.cpp
    test_core.cpp
    test_modding.cpp
    test_physics.cpp
    test_render_math.cpp
    test_scripting.cpp)

target_include_directories(sim_tests PRIVATE .)

target_link_libraries(sim_tests
    PRIVATE
        sim::core
        sim::modding
        sim::physics
        sim::render
        sim::scripting
        sim::vfs
        doctest::doctest)

sim_configure_target(sim_tests)

add_test(NAME sim_tests COMMAND sim_tests)
# Generous timeout: the Lua sandbox tests must never hang on a runaway script.
set_tests_properties(sim_tests PROPERTIES TIMEOUT 300)

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
    # Approved references live in tests/golden/<driver>/, keyed by the Vulkan
    # driver that produced them (lavapipe in CI, SwiftShader as an alternative).
    # While a reference has not been approved yet, the verification test returns
    # exit code 77 and ctest reports it as SKIPPED (neither pass nor fail).
    set(SIM_GOLDEN_DRIVER "lavapipe" CACHE STRING "Vulkan driver whose golden references are used")
    set(_golden "${CMAKE_SOURCE_DIR}/tests/golden/${SIM_GOLDEN_DRIVER}")
    set(_candidates "${CMAKE_SOURCE_DIR}/screenshots/golden_candidates")
    # Static render-test scenes (fixed cameras, no physics) and physics scenes
    # (deterministically stepped 1.5 s before capture, overlay included).
    set(_render_scenes default grid pyramid ramp orbit tower checker rows)
    set(_physics_scenes drop oscillator pendulum double_pendulum chain dzhanibekov gyro masses)
    set(_scenes ${_render_scenes} ${_physics_scenes})
    set(_golden_tests "")
    foreach(_scene IN LISTS _scenes)
        set(_test "sim_golden_${_scene}")
        add_test(NAME ${_test}
                 COMMAND $<TARGET_FILE:sim> --headless --frames 2 --width 640 --height 360
                         --scene ${_scene} --verify-golden --golden-dir "${_golden}" --semantic)
        set_tests_properties(${_test} PROPERTIES SKIP_RETURN_CODE 77)
        list(APPEND _golden_tests ${_test})
    endforeach()

    # The instanced debug-draw overlay (ground grid + world axes on the `grid`
    # scene) has its own reference so a regression in the instancing pipeline is
    # caught. No semantic probes: the overlay is decorative.
    add_test(NAME sim_golden_debug_overlay
             COMMAND $<TARGET_FILE:sim> --headless --frames 2 --width 640 --height 360
                     --scene grid --debug-draw --verify-golden --golden-dir "${_golden}")
    set_tests_properties(sim_golden_debug_overlay PROPERTIES SKIP_RETURN_CODE 77)
    list(APPEND _golden_tests sim_golden_debug_overlay)

    # Absolute screen-orientation test: the `orientation` harness scene keeps
    # its off-axis boxes and horizon check enabled even while the golden
    # references are still pending approval, so the RHI viewport flip and the
    # winding convention stay pinned by a test that always runs.
    add_test(NAME sim_semantics_orientation
             COMMAND $<TARGET_FILE:sim> --headless --frames 1 --width 640 --height 360
                     --scene orientation --semantic)
    set(_gpu_tests sim_headless_default sim_headless_resize sim_headless_validation sim_contact_sheet ${_golden_tests}
                   sim_semantics_orientation)
    set_tests_properties(${_gpu_tests} PROPERTIES LABELS gpu WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

    # Regenerate golden candidates for the selected driver (run manually after
    # intentional changes). Candidates go to screenshots/golden_candidates/,
    # never to tests/golden/: they are only committed once a human approves
    # them. Approval = copying the candidate into tests/golden/<driver>/.
    #   cmake --build <build> --target update_golden
    add_custom_target(update_golden
        COMMAND ${CMAKE_COMMAND} -E echo "Regenerating golden candidates into ${_candidates}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        VERBATIM)
    foreach(_scene IN LISTS _scenes)
        add_custom_command(TARGET update_golden POST_BUILD
            COMMAND $<TARGET_FILE:sim> --headless --frames 1 --width 640 --height 360
                    --scene ${_scene} --update-golden --golden-dir "${_candidates}" --semantic
            VERBATIM)
    endforeach()
    add_custom_command(TARGET update_golden POST_BUILD
        COMMAND $<TARGET_FILE:sim> --headless --frames 1 --width 640 --height 360
                --scene grid --debug-draw --update-golden --golden-dir "${_candidates}"
        VERBATIM)
    add_custom_command(TARGET update_golden POST_BUILD
        COMMAND $<TARGET_FILE:sim> --headless --frames 1 --width 640 --height 360
                --scene orientation --update-golden --golden-dir "${_candidates}" --semantic
        VERBATIM)
    if(SIM_VULKAN_ICD)
        # The Vulkan loader on Windows rejects forward slashes in the ICD path.
        file(TO_NATIVE_PATH "${SIM_VULKAN_ICD}" _icd_native)
        set_tests_properties(${_gpu_tests} PROPERTIES ENVIRONMENT "VK_ICD_FILENAMES=${_icd_native}")
    endif()
endif()
