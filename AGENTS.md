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

Windows + presets CMake (MSVC). No hay `cmake`/`cl`/`ninja`/`ctest` en el PATH:
hay que pasar por `vcvars64.bat` de Visual Studio. Locally `VCPKG_ROOT` apunta a
un vcpkg y todos los presets lo usan (`VCPKG_MANIFEST_MODE` ON). El SHADER
current slangc se resuelve con `SIM_SLANGC` o el env `SIM_SLANGC`.

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat"
set "VCPKG_ROOT=C:\dev\vcpkg"
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release --target sim       # app
cmake --build --preset windows-msvc-release --target sim_tests # suite
```

## Tests

- Tests de CPU (física, core, modding y matemática de render):
  `build\windows-msvc-release\bin\sim_tests.exe` o
  `ctest --test-dir build\windows-msvc-release -R sim_tests -V`.
- Tests GPU (headless, requieren ICD Vulkan + `-DSIM_ENABLE_GPU_TESTS=ON`):
  con Mesa lavapipe,
  `set VK_ICD_FILENAMES=C:\Programas\Car Simulator\tools\mesa\x64\lvp_icd.x86_64.json`
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