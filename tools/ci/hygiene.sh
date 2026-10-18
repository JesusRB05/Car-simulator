#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Repository hygiene gate. Fails if tracked files contain personal data or
# local runtime state, or if source and build files are missing the SPDX
# licence header. Meant to run from the repository root in CI.
set -euo pipefail

fail=0
note() { printf '  %s\n' "$1"; }
err() {
    printf 'error: %s\n' "$1" >&2
    fail=1
}

echo "hygiene: forbidden tracked files"
forbidden=$(git ls-files | grep -E '(^|/)(config\.toml|imgui\.ini)$|(^|/)userdata/|\.log$|^[^/]*\.png$' || true)
if [[ -n "$forbidden" ]]; then
    while IFS= read -r f; do
        err "tracked local/runtime file must not be committed: $f"
    done <<<"$forbidden"
else
    note "none"
fi

echo "hygiene: personal paths in tracked files"
# Absolute home directories for Windows, macOS and Linux users. This script is
# excluded because it necessarily contains the pattern it looks for.
personal=$(git grep -n -I -E 'C:\\Users\\|C:/Users/|/home/[A-Za-z0-9._-]+/|/Users/[A-Za-z0-9._-]+/' -- . \
    ':(exclude)tools/ci/hygiene.sh' || true)
if [[ -n "$personal" ]]; then
    while IFS= read -r line; do
        err "personal path found: $line"
    done <<<"$personal"
else
    note "none"
fi

echo "hygiene: SPDX headers"
# Source and build files must carry the SPDX identifier on the first or second
# line (the second line allows a shebang or a Windows @echo off).
header_ok=1
while IFS= read -r f; do
    first=$(sed -n '1p' "$f")
    second=$(sed -n '2p' "$f")
    if [[ "$first" != *"SPDX-License-Identifier"* && "$second" != *"SPDX-License-Identifier"* ]]; then
        err "missing SPDX licence header: $f"
        header_ok=0
    fi
done < <(git ls-files | grep -E '\.(c|cc|cpp|cxx|h|hpp|hxx|inl|ipp|slang|ps1|bat|sh|cmake|yml|yaml)$|(^|/)CMakeLists\.txt$' || true)
[[ "$header_ok" -eq 1 ]] && note "all present"

echo "hygiene: editor JSON schemas"
# The schemas in docs/schemas/ must stay in step with the descriptors in
# content/ and mods/ (see docs/MOD_FORMAT.md section 13).
if command -v python3 >/dev/null 2>&1; then
    python3 tools/ci/check_schemas.py || fail=1
else
    err "python3 is required to check the JSON schemas"
fi

echo "hygiene: large art through Git LFS"
# Every model and every content texture over 1 MiB must be tracked by Git LFS,
# so a clone stays small (see .gitattributes and docs/CONTRIBUTING.md). A file
# added without the filter shows up here as a plain blob.
lfs_ok=1
while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    need_lfs=0
    case "$f" in
        *.glb) need_lfs=1 ;;
        *.png | *.jpg | *.jpeg)
            size=$(wc -c <"$f" 2>/dev/null || echo 0)
            if [[ "$size" -gt 1048576 ]]; then need_lfs=1; fi
            ;;
    esac
    if [[ "$need_lfs" -eq 1 ]]; then
        filter=$(git check-attr filter -- "$f" | sed 's/.*: //')
        if [[ "$filter" != "lfs" ]]; then
            err "large art is not tracked by Git LFS: $f"
            lfs_ok=0
        fi
    fi
done < <(git ls-files 'content/**/*.glb' 'content/**/*.png' 'content/**/*.jpg' 'content/**/*.jpeg' || true)
[[ "$lfs_ok" -eq 1 ]] && note "all models and large content textures use LFS"

if [[ "$fail" -ne 0 ]]; then
    echo "hygiene: FAILED" >&2
    exit 1
fi
echo "hygiene: OK"
