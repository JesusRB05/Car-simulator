# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_rhi STATIC
    src/Conversions.cpp
    src/Device.cpp
    src/Swapchain.cpp
    src/Resources.cpp
    src/Pipeline.cpp
    src/CommandList.cpp
    src/Frame.cpp
    src/GpuTimer.cpp
    src/Vma.cpp)
add_library(sim::rhi ALIAS sim_rhi)

target_include_directories(sim_rhi PUBLIC include)
target_link_libraries(sim_rhi
    PUBLIC
        sim::core
        volk::volk
        GPUOpen::VulkanMemoryAllocator)
sim_configure_target(sim_rhi)
