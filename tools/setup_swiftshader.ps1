# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
#
# Downloads a pinned, prebuilt software Vulkan driver for local "gpu" tests so
# that the headless renderer can be exercised on a machine without a GPU.
#
# On Windows this fetches Mesa's lavapipe (the CPU Vulkan implementation) from
# the "mesa-dist-win" project. The archive is verified against a pinned SHA-256.
# A tiny standalone 7-Zip extractor (7zr.exe) is downloaded too, so no system
# installation is required.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools/setup_swiftshader.ps1
#   powershell -ExecutionPolicy Bypass -File tools/setup_swiftshader.ps1 -PrintEnvOnly
#
# Afterwards, configure the tests with the printed ICD path, e.g.:
#   cmake --preset windows-msvc-release -DSIM_ENABLE_GPU_TESTS=ON `
#         -DSIM_VULKAN_ICD="<printed path>"
#   ctest --test-dir build/windows-msvc-release -L gpu

#Requires -Version 5.1
[CmdletBinding()]
param(
    [string]$Dest = "",
    [string]$Version = "26.2.4",
    [string]$Sha256 = "351fc8c8b695878ffb3eaa044b3ead08672a48b1a045e3c3e3975811df0f6695",
    [switch]$PrintEnvOnly
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrEmpty($Dest))
{
    $Dest = Join-Path (Split-Path -Parent $PSCommandPath) "mesa"
}

$asset = "mesa3d-$Version-release-msvc.7z"
$url = "https://github.com/pal1000/mesa-dist-win/releases/download/$Version/$asset"
$sevenZipUrl = "https://www.7-zip.org/a/7zr.exe"

function Get-SoftwareIcd
{
    param([string]$Root)
    if (-not (Test-Path -LiteralPath $Root))
    {
        return $null
    }
    $json = Get-ChildItem -LiteralPath $Root -Recurse -Filter "lvp_icd*.json" -File -ErrorAction SilentlyContinue |
        Select-Object -First 1
    return $json
}

$existing = Get-SoftwareIcd -Root $Dest
if ($PrintEnvOnly -or $existing)
{
    if (-not $existing)
    {
        Write-Error "No ICD found under '$Dest'. Run without -PrintEnvOnly first."
    }
    $native = $existing.FullName
    Write-Host "VK_ICD_FILENAMES=$native"
    Write-Host "Add this directory to PATH: $($existing.DirectoryName)"
    Write-Host "Run: ctest --test-dir <build> -L gpu"
    exit 0
}

New-Item -ItemType Directory -Path $Dest -Force | Out-Null
$tmp = Join-Path ([System.IO.Path]::GetTempPath()) "sim-mesa-$Version"
New-Item -ItemType Directory -Path $tmp -Force | Out-Null
$archive = Join-Path $tmp $asset
$sevenZip = Join-Path $tmp "7zr.exe"

if (-not (Test-Path -LiteralPath $sevenZip))
{
    Write-Host "Downloading 7zr.exe (standalone 7-Zip extractor)..."
    Invoke-WebRequest -Uri $sevenZipUrl -OutFile $sevenZip -UseBasicParsing
}

Write-Host "Downloading $asset ..."
Invoke-WebRequest -Uri $url -OutFile $archive -UseBasicParsing

$actual = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actual -ne $Sha256.ToLowerInvariant())
{
    Remove-Item -LiteralPath $archive -Force
    Write-Error "SHA-256 mismatch for $asset (got $actual, expected $Sha256)."
}

Write-Host "Extracting to $Dest ..."
& $sevenZip x $archive "-o$Dest" -y | Out-Null

$json = Get-SoftwareIcd -Root $Dest
if (-not $json)
{
    Write-Error "Extraction succeeded but no lvp_icd*.json was found under '$Dest'."
}

$native = $json.FullName
Write-Host ""
Write-Host "Software Vulkan ICD ready."
Write-Host "VK_ICD_FILENAMES=$native"
Write-Host "Add this directory to PATH: $($json.DirectoryName)"
Write-Host ""
Write-Host "Configure tests with:"
Write-Host "  cmake --preset windows-msvc-release -DSIM_ENABLE_GPU_TESTS=ON -DSIM_VULKAN_ICD=`"$native`""
Write-Host "  ctest --test-dir build/windows-msvc-release -L gpu"
