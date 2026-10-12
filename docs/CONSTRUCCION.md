# Compilación

## Requisitos

| Herramienta | Versión mínima | Notas |
|---|---|---|
| CMake | 3.25 | El proyecto usa presets de CMake (`CMakePresets.json`, versión 6) |
| Compilador C++ | C++20 | MSVC 2022+ / GCC 13+ / Clang 16+ |
| vcpkg | última | Modo manifiesto (`vcpkg.json`) |
| Ninja | cualquiera | Los presets lo usan como generador |
| Vulkan | headers | Se instalan vía vcpkg; **no** hace falta el SDK completo |
| Driver Vulkan | 1.3 | Solo en tiempo de ejecución |

`slangc` (compilador de shaders) se descarga automáticamente con
`FetchContent` la primera vez que se configura, salvo que indiques una copia
local.

## vcpkg

El proyecto usa **modo manifiesto**: no hay que instalar dependencias a mano.
`CMakePresets.json` pasa `CMAKE_TOOLCHAIN_FILE` a partir de la variable de
entorno `VCPKG_ROOT`, así que debes definirla:

```powershell
# Windows PowerShell
$env:VCPKG_ROOT = "C:\ruta\a\vcpkg"
```

```bash
# Linux / macOS
export VCPKG_ROOT="$HOME/vcpkg"
```

El *baseline* está fijado en `vcpkg.json`
(`builtin-baseline`), de modo que todo el mundo instala exactamente las mismas
versiones.

### Dependencias

`cgltf`, `doctest`, `eigen3`, `glm`, `imgui`, `implot`, `ktx`, `luajit`,
`meshoptimizer`, `miniaudio`, `openxr-loader`, `physfs`, `sdl3` (con la feature
`vulkan`), `sol2`, `spdlog`, `stb`, `tomlplusplus`, `tracy`, `vulkan-headers`,
`vulkan-memory-allocator`, `volk`.

El cargador de Vulkan **no** se enlaza: `volk` resuelve `vulkan-1.dll` /
`libvulkan.so` en tiempo de ejecución.

## Presets

### Windows (MSVC + Ninja)

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

### Linux (GCC o Clang)

```bash
cmake --preset linux-gcc-debug
cmake --build --preset linux-gcc-debug
ctest --test-dir build/linux-gcc-debug --output-on-failure
```

Análogos: `linux-gcc-release`, `linux-clang-debug`, `linux-clang-release`.

## Slang

Si no hay `slangc` en el `PATH`, CMake descarga una versión precompilada de
Slang (`v2026.19`) mediante `FetchContent`. Puedes usar una copia local con:

```powershell
cmake --preset windows-msvc-debug -DSIM_SLANGC=C:\ruta\slang\bin\slangc.exe
```

o definiendo la variable de entorno `SIM_SLANGC`.

La función `sim_slang_shader(objetivo shader entrada etapa)` compila cada shader
`.slang` a SPIR-V en `${CMAKE_BINARY_DIR}/shaders/` y lo copia junto al
ejecutable. Los shaders actuales generan `mesh.vertex.spv` y
`mesh.fragment.spv`.

## Opciones de CMake

| Opción | Por defecto | Descripción |
|---|---|---|
| `SIM_BUILD_TESTS` | `ON` | Compila la suite de tests (doctest) |
| `SIM_WARNINGS` | `ON` | Conjunto estricto de avisos (`/W4`, `-Wall -Wextra`) |
| `SIM_WARNINGS_AS_ERRORS` | `OFF` | Trata los avisos como errores |
| `SIM_ENABLE_PROFILING` | `OFF` | Enlaza y activa Tracy |
| `SIM_ENABLE_SLANG` | `ON` | Compila shaders `.slang` a SPIR-V |
| `SIM_ENABLE_OPENXR` | `ON` | Integra el cargador de OpenXR |
| `SIM_ENABLE_FSR` | `OFF` | Descarga las fuentes de AMD FSR (aún sin usar) |

Pasarlas, por ejemplo:

```powershell
cmake --preset windows-msvc-debug -DSIM_BUILD_TESTS=OFF -DSIM_ENABLE_PROFILING=ON
```

## Artefactos

Con el layout por defecto, todos los binarios van a un único directorio:

```
build/<preset>/bin/sim.exe          (sim en Linux)
build/<preset>/bin/sim_tests.exe
build/<preset>/bin/shaders/*.spv
build/<preset>/lib/*.lib            (librerías estáticas de los módulos)
```

## Problemas frecuentes

- **`VCPKG_ROOT` no definido**: los presets fallan al no encontrar el toolchain
  de vcpkg. Define la variable de entorno.
- **`slangc` no encontrado**: deja que CMake lo descargue, o pasa
  `-DSIM_SLANGC=...`.
- **No arranca / pantalla negra**: comprueba que el driver soporta Vulkan 1.3 y
  que el registro de validación (en Debug) no reporta errores.
- **`sim.exe` no encuentra `content/base`**: ejecútalo desde la raíz del
  repositorio o desde un directorio en cuya jerarquía esté `content/base`.
