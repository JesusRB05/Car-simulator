# Car Simulator

Simulador de conducción de código abierto, mantenido por la comunidad, con un
**motor de física propio** y un **renderizador Vulkan propio**. No se usan
motores de física ni de render de terceros (Jolt, Bullet, PhysX, bgfx,
Diligent, Filament, ...): la física y el renderizado están escritos desde cero.

> Versión actual: **0.1.0** (hito inicial). El objetivo de este hito es dejar
> lista una base sólida y bien documentada, no un juego terminado.

## Características de esta base

- **Núcleo (core)**: reloj de paso fijo, sistema de trabajos, configuración
  TOML tipada, log con spdlog, utilidades matemáticas, aserciones.
- **Plataforma**: ventana SDL3, entrada de teclado, volantes/mandos/joysticks
  con reconexión en caliente, creación de la superficie Vulkan (el RHI no ve
  SDL nunca).
- **VFS**: sistema de ficheros virtual sobre PhysicsFS. Todo el contenido se
  lee por aquí; el contenido oficial y los mods viven en el mismo árbol virtual.
- **Modding**: manifiestos `mod.toml`, resolución de dependencias con rangos de
  versión (semver), orden de carga topológico determinista, montaje en el VFS.
- **Scripting**: sandbox de LuaJIT + sol2 con presupuesto de instrucciones.
- **Plugins nativos**: ABI en C puro, versionada y con evolución de struct
  controlada por `struct_size`.
- **RHI**: capa fina sobre Vulkan (volk + VMA), instancia/dispositivo,
  swapchain, buffers, imágenes, pipelines, listas de comandos, frames en vuelo.
- **Render**: cámara en perspectiva, escena plana, `RenderGraph` lineal, shader
  Slang compilado a SPIR-V en tiempo de build.
- **Física**: cuerpos rígidos, broadphase por AABB, estrechamiento
  esfera/esfera, esfera/plano, caja/plano y esfera/caja, solver de impulsos
  secuenciales, integración semi-implícita de Euler.
- **Vehículo**: modelo de bicicleta cinemático determinista con ruedas.
- **Telemetría**: histórico de tiempos de frame/physics/render/UI y exportación
  CSV.
- **Audio**: envoltorio mínimo sobre miniaudio.
- **Red**: esqueleto de la futura capa multijugador (sin sockets aún).

## Requisitos

- **CMake** >= 3.25
- **Compilador C++20**:
  - **MSVC / Visual Studio 2022** con el conjunto de herramientas **v143** (el
    estable) o superior; también se ha probado con builds recientes de VS. Usa
    el *x64 Native Tools Command Prompt* o ejecuta `vcvars64.bat` antes de
    configurar.
  - **GCC 13+** o **Clang 16+**.
- **vcpkg** (manifest mode)
- **Ninja** (recomendado; los presets lo usan)
- **Slang** (`slangc`): se descarga automáticamente vía `FetchContent` si no
  se indica una copia local
- **Vulkan SDK**: **no** es necesario para compilar (solo se usan
  `vulkan-headers` y `volk`); sí se necesita un **driver Vulkan** en tiempo de
  ejecución

## Compilación rápida (Windows / MSVC)

```powershell
# 1) Clona e instala vcpkg una vez y define VCPKG_ROOT
$env:VCPKG_ROOT = "C:\ruta\a\vcpkg"

# 2) Configura (la primera vez descarga e instala las dependencias)
cmake --preset windows-msvc-debug

# 3) Compila
cmake --build --preset windows-msvc-debug

# 4) Ejecuta los tests
ctest --test-dir build/windows-msvc-debug --output-on-failure
```

Binarios generados:

```
build/windows-msvc-debug/bin/sim.exe
build/windows-msvc-debug/bin/sim_tests.exe
```

Para Release:

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release --output-on-failure
```

Si quieres usar una copia local de Slang en lugar de la descargada:

```powershell
cmake --preset windows-msvc-debug -DSIM_SLANGC=C:\ruta\slang\bin\slangc.exe
```

En Linux se ofrecen los presets `linux-gcc-*` y `linux-clang-*`.

Instrucciones detalladas en [`docs/CONSTRUCCION.md`](docs/CONSTRUCCION.md).

## Tests de GPU sin GPU (headless)

El simulador puede renderizar **fuera de pantalla**, sin ventana ni swapchain,
mediante un driver Vulkan software. Esto permite ejecutar los tests marcados con
la etiqueta `gpu` (comparación con imágenes de referencia, verificación de
píxeles, etc.) en un equipo sin GPU y en CI.

Primero descarga el driver software (lavapipe) **pinnado y con SHA-256
verificado**:

```powershell
# Windows
powershell -ExecutionPolicy Bypass -File tools/setup_swiftshader.ps1
```

```bash
# Linux (usa el paquete mesa-vulkan-drivers, o lo instala con apt)
./tools/setup_swiftshader.sh
```

El script imprime la ruta del ICD. Configura y ejecuta los tests:

```powershell
cmake --preset windows-msvc-release -DSIM_ENABLE_GPU_TESTS=ON `
      -DSIM_VULKAN_ICD="C:\Programas\Car Simulator\tools\mesa\x64\lvp_icd.x86_64.json"
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release -L gpu --output-on-failure
```

> En Windows, `VK_ICD_FILENAMES` **debe** usar barras invertidas; el cargador
> rechaza las barras normales. Añade además el directorio del ICD al `PATH`.

Modo de ejecución manual del renderer headless:

```powershell
./build/windows-msvc-release/bin/sim.exe --headless --frames 60 `
    --scene default --screenshot captura.png
```

El runner headless también soporta `--verify-golden`/`--update-golden`
(con `--golden-dir`), `--semantic`, `--contact-sheet` y `--debug-draw`; los tests
`gpu` incluyen una referencia por escena y otra del *debug draw*
(`tests/golden/grid_debug.png`).

## Ejecución

`sim.exe` busca la raíz del proyecto (el directorio que contiene
`content/base`) partiendo del directorio de trabajo y del directorio del
ejecutable. Desde ahí monta el contenido, lee la configuración y arranca.

```powershell
./build/windows-msvc-debug/bin/sim.exe
```

### Controles por defecto

| Acción | Tecla |
|---|---|
| Acelerar | `W` / `↑` |
| Frenar | `S` / `↓` |
| Girar | `A` / `D` / `←` / `→` |
| Freno de mano | `Espacio` |
| Salir | `Esc` |

Cámara y depuración: `C` alterna los modos **Chase / Orbit / Free** (arrastrar con
el botón derecho para mirar, rueda para acercar en Orbit; `WASD`/`Q`/`E` para
volar en Free), y `G` muestra u oculta la rejilla de depuración instanciada.

También funciona con mandos, volantes y pedales detectados por SDL3.

## Estructura del repositorio

```
apps/sim/            Aplicación ejecutable (bucle principal, ImGui, interpolación)
cmake/               Módulos CMake (helpers, dependencias, Slang, ImGui, FSR)
content/base/        Contenido oficial (CC BY-SA 4.0)
mods/                Mods de ejemplo
plugins/example_native/  Plugin nativo de ejemplo
src/
  core/              Reloj, jobs, configuración, log, matemáticas
  platform/          SDL3: contexto, ventana, entrada
  vfs/               Sistema de ficheros virtual (PhysicsFS)
  modding/           Manifiestos, versiones, dependencias, gestor de mods
  scripting/         Sandbox Lua (LuaJIT + sol2) y host de scripts
  plugin_api/        ABI de plugins nativos + cargador
  rhi/               Capa de render Vulkan propia
  render/            Escena, cámara, mallas, shaders (Slang)
  physics/           Cuerpos rígidos, colisiones, solver
  vehicle/           Modelo del vehículo
  telemetry/         Métricas de frame + export CSV
  audio/             miniaudio
  net/               Esqueleto de red
tests/               Suite doctest
docs/                Documentación (español)
```

## Documentación

- [Arquitectura](docs/ARQUITECTURA.md)
- [Compilación](docs/CONSTRUCCION.md)
- [Física](docs/FISICA.md)
- [Renderizado](docs/RENDERIZADO.md)
- [Sistema de mods](docs/MODS.md)
- [Scripting (Lua)](docs/SCRIPTS.md)
- [Plugins nativos](docs/PLUGINS.md)
- [Contenido y licencias](docs/CONTENIDO.md)
- [Decisiones de diseño y limitaciones](docs/DECISIONES.md)
- [Contribuir](docs/CONTRIBUIR.md)

## Licencia

- Código del motor: **GNU GPL v3.0 o posterior**, con una **excepción de
  enlazado** (ver [`LICENSE`](LICENSE)).
- Contenido oficial (`content/base`): **CC BY-SA 4.0**
  (ver [`content/base/LICENSE.txt`](content/base/LICENSE.txt)).

No se usan marcas, coches ni circuitos reales a propósito.
