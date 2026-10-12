#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Sets up a software Vulkan driver for local "gpu" tests on machines without a
# GPU. On Linux this uses Mesa's lavapipe (the CPU Vulkan implementation), which
# ships in the distribution's mesa-vulkan-drivers package. If it is missing and
# apt is available, it is installed.
#
# Usage:
#   ./tools/setup_swiftshader.sh          # locate or install lavapipe, print env
#   ./tools/setup_swiftshader.sh --print  # only print the ICD env line
#
# Afterwards, configure the tests with the printed ICD path, e.g.:
#   cmake --preset linux-gcc-release -DSIM_ENABLE_GPU_TESTS=ON \
#         -DSIM_VULKAN_ICD="<printed path>"
#   ctest --test-dir build/linux-gcc-release -L gpu

set -euo pipefail

print_only=0
if [[ "${1:-}" == "--print" ]]; then
    print_only=1
fi

find_icd()
{
    for candidate in \
        /usr/share/vulkan/icd.d/lvp_icd.x86_64.json \
        /usr/share/vulkan/icd.d/lvp_icd.i686.json \
        /usr/share/vulkan/icd.d/lvp_icd.aarch64.json
    do
        if [[ -f "$candidate" ]]; then
            echo "$candidate"
            return 0
        fi
    done
    # Fall back to any lavapipe ICD the distribution may have named differently.
    icd=$(find /usr/share/vulkan/icd.d -name 'lvp_icd*.json' 2>/dev/null | head -n1 || true)
    if [[ -n "$icd" ]]; then
        echo "$icd"
        return 0
    fi
    return 1
}

if ! icd=$(find_icd); then
    if [[ "$print_only" -eq 1 ]]; then
        echo "lavapipe ICD not found. Run without --print to install it." >&2
        exit 1
    fi
    if command -v apt-get >/dev/null 2>&1; then
        echo "Installing mesa-vulkan-drivers (lavapipe)..."
        if [[ "$(id -u)" -eq 0 ]]; then
            apt-get update
            apt-get install -y mesa-vulkan-drivers
        else
            sudo apt-get update
            sudo apt-get install -y mesa-vulkan-drivers
        fi
        icd=$(find_icd) || {
            echo "lavapipe still not found after install." >&2
            exit 1
        }
    else
        echo "No lavapipe ICD and no apt-get. Install Mesa's Vulkan drivers" >&2
        echo "(package 'mesa-vulkan-drivers') and retry." >&2
        exit 1
    fi
fi

echo "Software Vulkan ICD ready."
echo "VK_ICD_FILENAMES=$icd"
echo ""
echo "Configure tests with:"
echo "  cmake --preset linux-gcc-release -DSIM_ENABLE_GPU_TESTS=ON -DSIM_VULKAN_ICD=\"$icd\""
echo "  ctest --test-dir build/linux-gcc-release -L gpu"
