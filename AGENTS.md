# AGENTS.md

Guía para agentes que trabajan en este repositorio. Léela antes de tocar código.

## Proyecto

Simulador de conducción: motor de física propio (cuerpos rígidos, articulaciones),
render Vulkan (RHI pequeño + renderer), vehículo, app interactiva (`apps/sim`) con
ImGui/ImPlot, modding y plugin API. La documentación del proyecto está en `docs/`
(en español); las convenciones de código están en `docs/CONVENTIONS.md`.

## Convenciones esenciales

- Unidades SI, radianes, segundos, metros. Mundo diestro con `-Z` adelante
  (excepto el vehículo, que usa `+Z` a rumbo 0).
- Matrices **column-major** en GLM y **`column_major`** + `mul(M, v)` en Slang;
  se suben verbatim con `writeMatrix()`, sin transponer nunca.
- Profundidad inversa (reversed-Z): `GreaterEqual`, clear a `0.0f`.
- Física en doble precisión, paso fijo, determinista, `/fp:precise`, sin
  `-ffast-math`. No introducir `float` ni `dt` variable en el motor.
- C++17+, SPDX GPL-3.0-or-later en cada archivo, `clang-format`, documentación
  en español.

## Build

Windows builds use the CMake presets (MSVC + Ninja). `cmake`, `cl`, `ninja` and
`ctest` are usually not on the `PATH`; open a developer environment first.

- **Visual Studio 2022 or later** (Build Tools are enough) with the
  "MSVC x64/x86 build tools" component. Locate it with `vswhere` instead of
  hardcoding an edition or version path, then call
  `VC\Auxiliary\Build\vcvars64.bat` from that installation.
  `tools\dev-env.bat` does exactly this.
- **vcpkg**: set the `VCPKG_ROOT` environment variable to a stable clone (for
  example `C:\dev\vcpkg`, never a temporary folder). All presets take the
  toolchain from `$env{VCPKG_ROOT}` and run in manifest mode; `vcpkg.json` pins
  the `builtin-baseline`, so everyone gets the same dependency versions.
- **slangc**: if it is not on the `PATH`, CMake downloads a prebuilt Slang.
  To use a local copy, pass the `SIM_SLANGC` cache variable
  (`-DSIM_SLANGC=...\slangc.exe`) or set the `SIM_SLANGC` environment variable.
- **Non-English MSVC**: install the English language pack. The Windows presets
  set `VSLANG=1033`; without the English pack `cl /showIncludes` prints
  localized prefixes and Ninja silently loses header dependencies.

```bat
for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat"
rem or simply: call tools\dev-env.bat
set "VCPKG_ROOT=C:\dev\vcpkg"
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release --target sim       # app
cmake --build --preset windows-msvc-release --target sim_tests # suite
```

(Inside an interactive `cmd` prompt, write `%i` instead of `%%i`.)

## Tests

- Tests de CPU (física, core, modding y matemática de render):
  `build\windows-msvc-release\bin\sim_tests.exe` o
  `ctest --test-dir build\windows-msvc-release -R sim_tests -V`.
- Tests GPU (headless, requieren ICD Vulkan + `-DSIM_ENABLE_GPU_TESTS=ON`):
  con Mesa lavapipe,
  `set "VK_ICD_FILENAMES=%CD%\tools\mesa\x64\lvp_icd.x86_64.json"` (run from the repository root)
  y `tools\mesa\x64` en `PATH`; luego
  `ctest --test-dir build\windows-msvc-release -L gpu --output-on-failure`.
- Dorados: las candidatas se regeneran a mano con
  `cmake --build <build> --target update_golden` (escriben en
  `screenshots/golden_candidates/`) tras cambios intencionados, nunca para
  maquillar una regresión. Pasan a `tests/golden/<driver>/` solo tras aprobación
  humana; mientras no existan, los tests GPU de verificación se reportan como
  omitidos (`SKIP_RETURN_CODE 77`). Comparación con tolerancias realistas
  (máx. 1 % de píxeles distintos, PSNR ≥ 30 dB).

## Git

- Commits lógicos, mensajes estilo Conventional Commits en inglés, y **todos
  firmados con DCO** (`git commit -s`); ver `docs/CONTRIBUIR.md`.
- No hacer push automático si no hay credenciales; el remoto `origin` puede ser
  un placeholder.