# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Creates the labels and the initial roadmap issues on GitHub with the GitHub
# CLI. It is idempotent: labels are updated in place and an issue is skipped
# when one with the same title already exists.
#
# Usage (after `gh auth login`):
#   powershell -ExecutionPolicy Bypass -File tools/github/seed_issues.ps1
#   powershell -ExecutionPolicy Bypass -File tools/github/seed_issues.ps1 -DryRun

#Requires -Version 5.1
[CmdletBinding()]
param(
    [string]$Repo = "JesusRB05/Car-simulator",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

$labels = @(
    @{ name = "good first issue"; color = "7057ff"; description = "Good for newcomers" },
    @{ name = "help wanted"; color = "008672"; description = "Extra attention is welcome" },
    @{ name = "area: physics"; color = "d93f0b"; description = "Physics engine and vehicle dynamics" },
    @{ name = "area: rendering"; color = "1d76db"; description = "Vulkan renderer and visuals" },
    @{ name = "area: ui"; color = "fbca04"; description = "Menus, HUD and tooling UI" },
    @{ name = "area: input"; color = "0e8a16"; description = "Wheels, pedals, gamepads, force feedback" },
    @{ name = "area: audio"; color = "5319e7"; description = "Sound" },
    @{ name = "area: content"; color = "c5def5"; description = "Cars, tracks, skins" },
    @{ name = "area: modding"; color = "bfd4f2"; description = "Mods, Lua scripting, plugins" },
    @{ name = "area: build"; color = "ededed"; description = "Build system, CI, packaging" },
    @{ name = "documentation"; color = "0075ca"; description = "Docs and tutorials" },
    @{ name = "bug"; color = "d73a4a"; description = "Something is not working" },
    @{ name = "enhancement"; color = "a2eeef"; description = "New feature or request" }
)

$issues = @(
    @{
        title  = "Add a lap timer, sector times and a racing HUD"
        labels = @("enhancement", "area: ui", "help wanted")
        body   = "The drive scene has no lap timing. Add start/finish and sector gates from the track layout, current/last/best lap, delta to best, and a HUD with speed, gear, RPM and pedal bars.`n`nDone when: a full lap on Sierra Verde produces correct sector and lap times, with unit tests for the timer (gate crossing, invalid laps)."
    },
    @{
        title  = "Import barrier collision volumes from circuit models"
        labels = @("enhancement", "area: physics", "help wanted")
        body   = "Imported circuits are drawn and drive on their collision mesh, but the walls are visual only. Import barrier geometry as static collision meshes and collide the chassis against it.`n`nDone when: a car driven into a wall at a known speed stops or deflects without tunnelling, with tests and an unchanged determinism hash for scenes without walls."
    },
    @{
        title  = "Vehicle audio: engine, tyres and wind"
        labels = @("enhancement", "area: audio", "help wanted")
        body   = "There is no sound yet. Add RPM-driven engine audio, tyre squeal from slip, wind and collision hits. Use procedural synthesis or CC0/CC-BY assets recorded in docs/CONTENT.md.`n`nAudio must never feed back into the physics."
    },
    @{
        title  = "Car setup screen: tyre pressure, aero, gearing and dampers"
        labels = @("enhancement", "area: ui", "area: physics")
        body   = "Let the player tune a car in the garage and save named setups, following docs/schemas/setup.schema.json. Applying a setup must change the vehicle model parameters at build time."
    },
    @{
        title  = "Sample normal, metallic-roughness and occlusion maps in the shader"
        labels = @("enhancement", "area: rendering", "good first issue")
        body   = "The glTF loader reads these maps but the shader does not sample them yet. Wire them into the PBR pass (Slang) and update the golden images after review.`n`nStarting points: src/render, docs/RENDERING.md."
    },
    @{
        title  = "Full force feedback for wheels on Windows (DirectInput)"
        labels = @("enhancement", "area: input", "help wanted")
        body   = "Force feedback currently sends rumble and LED only. Add constant-force and spring/damper/friction effects driven by the steering column torque. Needs a real wheel to test."
    },
    @{
        title  = "Validate the tyre model against published data"
        labels = @("area: physics", "help wanted")
        body   = "Compare the Magic Formula implementation against published tyre data or a reference implementation and add tests with explicit tolerances. See docs/PHYSICS_PLAN.md, section 10."
    },
    @{
        title  = "Tyre thermal model and wear"
        labels = @("enhancement", "area: physics")
        body   = "Add tyre temperature (surface/core), pressure change and wear, with grip depending on temperature. Parameters come from data files; add validation tests."
    },
    @{
        title  = "Add a second fictional car (front-wheel-drive hatchback)"
        labels = @("area: content", "good first issue", "help wanted")
        body   = "Create a data-only car with a different layout from the Halvard GT3 to prove the engine expresses differences as data. See mods/example_mod and docs/MOD_FORMAT.md. No real brands."
    },
    @{
        title  = "Create a new original circuit"
        labels = @("area: content", "help wanted")
        body   = "Model an original circuit under a free licence, with a glTF model and a track.json layout. See content/base/tracks/sierra_verde and tools/asset_gen."
    },
    @{
        title  = "Weather and time of day"
        labels = @("enhancement", "area: rendering")
        body   = "Add a sun position / sky model and rain (wet surface friction through the existing surface data, brake and rain lights already exist)."
    },
    @{
        title  = "Replays and ghost laps"
        labels = @("enhancement", "area: physics", "area: ui")
        body   = "The physics is deterministic with snapshot/restore, which makes replays cheap. Record inputs and play them back, and race against a ghost of your best lap."
    },
    @{
        title  = "Linux packaging (AppImage or tarball)"
        labels = @("area: build", "help wanted")
        body   = "Provide a release artifact for Linux built in CI, including the content and shaders."
    },
    @{
        title  = "Improve the build documentation for a first-time contributor"
        labels = @("documentation", "good first issue")
        body   = "Follow docs/BUILDING.md on a clean machine and fix every gap, for example the vcpkg setup, the compiler flags, and the non-English MSVC note."
    },
    @{
        title  = "Add screenshots and a short video to the README"
        labels = @("documentation", "good first issue")
        body   = "Record a short capture of a lap and the menus (tools/capture_readme_shots.bat renders stills) and add them to the README."
    }
)

function Invoke-Gh {
    param([string[]]$GhArgs)
    if ($DryRun) { Write-Host "gh $($GhArgs -join ' ')"; return }
    & gh @GhArgs
    if ($LASTEXITCODE -ne 0) { throw "gh failed: $($GhArgs -join ' ')" }
}

foreach ($l in $labels) {
    Invoke-Gh @("label", "create", $l.name, "--repo", $Repo, "--color", $l.color, "--description", $l.description, "--force")
}

$existing = @()
if (-not $DryRun) {
    $existing = (gh issue list --repo $Repo --state all --limit 200 --json title | ConvertFrom-Json).title
}

foreach ($i in $issues) {
    if ($existing -contains $i.title) {
        Write-Host "skip (exists): $($i.title)"
        continue
    }
    $ghArgs = @("issue", "create", "--repo", $Repo, "--title", $i.title, "--body", $i.body)
    foreach ($label in $i.labels) { $ghArgs += @("--label", $label) }
    Invoke-Gh $ghArgs
}
