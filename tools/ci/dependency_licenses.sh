#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Generates a licence report for the dependencies installed by the vcpkg
# manifest (manifest mode). Usage:
#   tools/ci/dependency_licenses.sh <vcpkg_installed-dir> [output-file]
# It concatenates the copyright file shipped by every installed port and lists
# the declared versions, so the report is auditable before a release.
set -euo pipefail

installed_dir=${1:?usage: dependency_licenses.sh <vcpkg_installed-dir> [output-file]}
output=${2:-dependency-licenses.txt}

if [[ ! -d "$installed_dir" ]]; then
    echo "error: '$installed_dir' does not exist" >&2
    exit 1
fi

{
    echo "Dependency licence report"
    echo "========================="
    echo "Generated from: $installed_dir"
    echo
    echo "Declared dependencies (vcpkg.json):"
    cat vcpkg.json
    echo
    echo "Copyright files:"
} >"$output"

count=0
# Manifest mode keeps the installed tree (with each port's share/<port>/copyright)
# either directly under the installed dir or under its triplet subdirectory.
while IFS= read -r copyright; do
    echo >>"$output"
    echo "--------------------------------------------------------------------------------" >>"$output"
    echo "## $copyright" >>"$output"
    echo "--------------------------------------------------------------------------------" >>"$output"
    cat "$copyright" >>"$output"
    count=$((count + 1))
done < <(find "$installed_dir" -type f -iname 'copyright*' | sort)

if [[ "$count" -eq 0 ]]; then
    echo "error: no copyright files found under '$installed_dir'" >&2
    exit 1
fi

echo "Dependency licence report written to $output ($count packages)."
