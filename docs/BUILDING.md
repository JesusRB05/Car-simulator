# Building

## Requirements

| Tool | Minimum version | Notes |
|---|---|---|
| CMake | 3.25 | The project uses CMake presets (`CMakePresets.json`, version 6) |
| C++ compiler | C++20 | MSVC 2022+ / GCC 13+ / Clang 16+ |
| vcpkg | latest | Manifest mode (`vcpkg.json`) |
| Ninja | any | The presets use it as the generator |
| Vulkan | headers | Installed through vcpkg; the full SDK is **not** needed |
| Vulkan driver | 1.3 | Run time only |

`slangc` (the shader compiler) is downloaded automatically with
`FetchContent` the first time you configure, unless you point to a local copy.

## vcpkg

The project uses **manifest mode**: there is no need to install dependencies by
hand. `CMakePresets.json` passes `CMAKE_TOOLCHAIN_FILE` based on the
`VCPKG_ROOT` environment variable, so you must set it:

```powershell
# Windows PowerShell
$env:VCPKG_ROOT = "C:\dev\vcpkg"
```

```bash
# Linux / macOS
export VCPKG_ROOT="$HOME/vcpkg"
```

Point `VCPKG_ROOT` at a stable clone of vcpkg, never at a temporary folder.
`vcpkg.json` pins `builtin-baseline`
(`e456309491fd875487bf02b8a0dca76c1b1b00cc`), so every machine and CI resolve
the same port versions. To update dependencies, bump the baseline in a
dedicated commit.

### Dependencies

`cgltf`, `doctest`, `eigen3`, `glm`, `imgui`, `implot`, `ktx`, `luajit`,
`meshoptimizer`, `miniaudio`, `openxr-loader`, `physfs`, `sdl3` (with the
`vulkan` feature), `sol2`, `spdlog`, `stb`, `tomlplusplus`, `tracy`,
`vulkan-headers`, `vulkan-memory-allocator`, `volk`.

The Vulkan loader is **not** linked: `volk` resolves `vulkan-1.dll` /
`libvulkan.so` at run time.

## Presets

### Windows (MSVC + Ninja)

**Developer environment.** The presets need `cl`, `ninja`, `cmake` and `ctest`
on the `PATH`. Use Visual Studio 2022 or later (Build Tools are enough) with
the "MSVC x64/x86 build tools" component, located with `vswhere` rather than a
hardcoded path, and call `vcvars64.bat` from that installation.
`tools\dev-env.bat` does this for you:

```bat
call tools\dev-env.bat
set "VCPKG_ROOT=C:\dev\vcpkg"
```

Manual equivalent (in a `.bat` file; use `%i` in an interactive prompt):

```bat
for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat"
```

**Non-English MSVC.** Install the English language pack for Visual Studio.
The Windows presets set `VSLANG=1033`; without the English pack, the localized
`cl /showIncludes` prefix does not match what Ninja expects and Ninja silently
loses header dependencies (edits to headers stop triggering rebuilds).

```powershell
# Debug
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug
ctest --test-dir build/windows-msvc-debug --output-on-failure

# Release
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release --output-on-failure
```

### Linux (GCC or Clang)

```bash
cmake --preset linux-gcc-debug
cmake --build --preset linux-gcc-debug
ctest --test-dir build/linux-gcc-debug --output-on-failure
```

Likewise: `linux-gcc-release`, `linux-clang-debug`, `linux-clang-release`.

## Slang

If there is no `slangc` on the `PATH`, CMake downloads a prebuilt Slang
(`v2026.19`) through `FetchContent`. You can use a local copy with:

```powershell
cmake --preset windows-msvc-debug -DSIM_SLANGC=C:\path\to\slang\bin\slangc.exe
```

or by setting the `SIM_SLANGC` environment variable (`SIM_SLANGC` can be given
either as a CMake cache variable or as an environment variable).

The `sim_slang_shader(target shader entry stage)` function compiles each
`.slang` shader to SPIR-V in `${CMAKE_BINARY_DIR}/shaders/` and copies it next
to the executable. The current shaders produce `mesh.vertex.spv` and
`mesh.fragment.spv`.

## CMake options

| Option | Default | Description |
|---|---|---|
| `SIM_BUILD_TESTS` | `ON` | Builds the test suite (doctest) |
| `SIM_WARNINGS` | `ON` | Strict warning set (`/W4`, `-Wall -Wextra`) |
| `SIM_WARNINGS_AS_ERRORS` | `OFF` | Treats warnings as errors |
| `SIM_ENABLE_PROFILING` | `OFF` | Links and enables Tracy |
| `SIM_ENABLE_SLANG` | `ON` | Compiles `.slang` shaders to SPIR-V |
| `SIM_ENABLE_OPENXR` | `ON` | Integrates the OpenXR loader |
| `SIM_ENABLE_FSR` | `OFF` | Downloads the AMD FSR sources (not used yet) |

For example:

```powershell
cmake --preset windows-msvc-debug -DSIM_BUILD_TESTS=OFF -DSIM_ENABLE_PROFILING=ON
```

## Artifacts

With the default layout, all binaries go to a single directory:

```
build/<preset>/bin/sim.exe          (sim on Linux)
build/<preset>/bin/sim_tests.exe
build/<preset>/bin/shaders/*.spv
build/<preset>/lib/*.lib            (static libraries of the modules)
```

## Troubleshooting

- **`VCPKG_ROOT` not set**: the presets fail because they cannot find the vcpkg
  toolchain. Set the environment variable.
- **Header edits do not trigger rebuilds on Windows**: MSVC is running in a
  non-English language. Install the English language pack (see above).
- **`cl` / `ninja` not found**: run `call tools\dev-env.bat` first.
- **`slangc` not found**: let CMake download it, or pass `-DSIM_SLANGC=...`.
- **Does not start / black screen**: check that the driver supports Vulkan 1.3
  and that the validation layer (in Debug) reports no errors.
- **`sim.exe` cannot find `content/base`**: run it from the repository root or
  from a directory whose hierarchy contains `content/base`.
