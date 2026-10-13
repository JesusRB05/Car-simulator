@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 The Car Simulator contributors
rem
rem Opens an x64 MSVC developer environment in the current cmd session.
rem Locates Visual Studio 2022 or later (including Build Tools and previews)
rem with vswhere and calls vcvars64.bat from that installation.
rem
rem Usage (from cmd):  call tools\dev-env.bat
rem VCPKG_ROOT must point to a stable vcpkg clone (for example C:\dev\vcpkg).

set "SIM_VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%SIM_VSWHERE%" (
    echo error: vswhere.exe not found. Install Visual Studio 2022 or later.
    exit /b 1
)

set "SIM_VSINSTALL="
for /f "usebackq delims=" %%i in (`"%SIM_VSWHERE%" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "SIM_VSINSTALL=%%i"
if not defined SIM_VSINSTALL (
    echo error: no Visual Studio installation with the MSVC x64 tools was found.
    exit /b 1
)

rem Some vcvars extension scripts call vswhere without a path.
set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;%PATH%"
call "%SIM_VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
set "SIM_VSWHERE="
set "SIM_VSINSTALL="

if not defined VCPKG_ROOT (
    echo warning: VCPKG_ROOT is not set; the CMake presets need it.
)
exit /b 0
