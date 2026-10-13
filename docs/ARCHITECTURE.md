# Arquitectura

Este documento describe cómo está organizado el simulador, las convenciones que
sigue todo el código y cómo se comunican las distintas capas.

## Principios

1. **Sin motores de terceros** para física ni render. Se permiten bibliotecas de
   utilidad (Vulkan headers, VMA, volk, glm, SDL3, ...), pero el motor de
   física y el renderizador son propios.
2. **Unidades del SI** en todo el motor (metros, kilogramos, segundos,
   radianes). No hay unidades "de juego" escondidas.
3. **Precisión controlada**: las posiciones del mundo se guardan en `double`
   (`glm::dvec3`); lo local y lo que va a la GPU, en `float`.
4. **Determinismo**: mismo conjunto de entradas ⇒ misma secuencia de estados,
   con independencia de la velocidad de la máquina.
5. **Sistema de coordenadas**: diestro (right-handed), **+Y arriba**, **-Z
   adelante**.
6. **Separación estricta por capas**: cada módulo depende solo de los que están
   por debajo. La física no conoce el render; el RHI no conoce SDL.

## Capas y módulos

De abajo hacia arriba:

| Módulo | Responsabilidad | Depende de |
|---|---|---|
| `core` | Reloj de paso fijo, jobs, config TOML, log, matemáticas, aserciones | — |
| `platform` | SDL3: contexto, ventana, entrada, superficie Vulkan | `core` |
| `vfs` | Sistema de ficheros virtual (PhysicsFS) | `core` |
| `modding` | Manifiestos, versiones, dependencias, gestor de mods | `core`, `vfs` |
| `scripting` | Sandbox Lua y host de scripts | `core`, `vfs` |
| `plugin_api` | ABI de plugins nativos + cargador | `core`, `vfs` |
| `rhi` | Instancia/dispositivo, swapchain, recursos, pipelines | `core` |
| `render` | Escena, cámara, mallas, shaders, grafo de render | `core`, `rhi`, `vfs` |
| `physics` | Cuerpos rígidos, elementos de fuerza y articulaciones (solver PGS) | `core` |
| `vehicle` | Modelo del vehículo | `core`, `physics` |
| `telemetry` | Métricas de frame y exportación CSV | `core`, `vfs` |
| `audio` | miniaudio | `core`, `vfs` |
| `net` | Esqueleto de red | `core` |
| `apps/sim` | Aplicación: bucle principal, ImGui, física en hilo aparte | todos |

Cada módulo es una librería estática `sim::<nombre>` con sus cabeceras públicas
en `include/<nombre>/` y sus fuentes en `src/`.

## Bucle principal

El programa (`apps/sim`) sigue este orden:

```
main()
  └─ Application::run()
       ├─ initialise()
       │    ├─ SdlContext, Window, InputManager
       │    ├─ Config (config.toml del usuario)
       │    ├─ VirtualFileSystem
       │    ├─ ModManager::reload()      (monta contenido y mods)
       │    ├─ montar el directorio del ejecutable (shaders) por debajo
       │    ├─ LuaSandbox + ScriptHost, PluginLoader
       │    ├─ superficie Vulkan + Device + Swapchain + Frame
       │    ├─ Renderer, ImGuiLayer, Telemetry, AudioEngine, NetworkSystem
       │    ├─ escena inicial (suelo + cubo) y vehículo
       │    └─ startPhysicsThread()
       └─ bucle:
            ├─ pumpInput()              (SDL → InputManager/ImGui, teclas → atómicos)
            ├─ gatherSnapshots()        (consume el anillo, interpola)
            ├─ frame_->begin()          (espera slot, adquiere imagen)
            ├─ imgui_->beginFrame()
            ├─ updateScene()            (rellena Scene desde el snapshot)
            ├─ renderer_->renderScene()
            ├─ drawUi()                 (paneles de telemetría/entrada)
            ├─ imgui_->endFrame()
            ├─ frame_->submitAndPresent()
            └─ gestión de resize/cierre
```

## Hilo de física y snapshots

La simulación corre en un **hilo dedicado** con un paso fijo (por defecto
`1/1000 s`) para que el resultado no dependa de los FPS. El hilo de render
interpola entre los dos últimos estados para que el movimiento se vea suave.

```
Hilo de física                         Hilo de render
─────────────────                      ─────────────────
lee atómicos de entrada                escribe atómicos de entrada
bucle de paso fijo                     cada frame:
  ├─ FixedStepClock.advance(dt)          ├─ pop() todos los snapshots nuevos
  ├─ vehicle.step(dt)                    ├─ previous_ = penúltimo
  ├─ world.step(dt)                      ├─ current_  = último
  └─ push(SimSnapshot)  ──── anillo ──►  └─ alpha = clamp((now - current.wallTime)/dt)
                                              interpola posición/heading/ruedas
```

- El canal de entrada son varios `std::atomic` (`throttle`, `brake`, `steer`,
  `handbrake`).
- El canal de salida es `SnapshotRing`: un anillo **lock-free SPSC** de 128
  `SimSnapshot`, con índice de escritura/lectura atómicos.
- `SimSnapshot` lleva: tiempo de simulación, tiempo de reloj real, número de
  pasos, posición del vehículo (`dvec3`), rumbo, velocidad, rpm, ángulo de
  dirección y el giro de las 4 ruedas.
- El reloj de render usa `FrameClock` (basado en `std::chrono::steady_clock`).

## Determinismo

- Paso fijo en física; nada de `dt` variable dentro de la simulación.
- MSVC: `/fp:precise`. GCC/Clang: `-ffp-contract=off`. Nunca `-ffast-math`.
- El orden de evaluación de la física es siempre el mismo (sin paralelismo
  dentro del paso). El `JobSystem` se reserva para trabajo que **no** afecta a
  la simulación (carga de assets, escaneo de mods).

## Configuración y VFS

- La **configuración del usuario** es un TOML leído/escrito por
  `core::Config` (claves con notación de puntos, p. ej. `mods.active`). Este
  archivo **no** forma parte del VFS.
- Todo el **contenido del juego** (texturas, mallas, scripts, sonidos) se lee a
  través de `vfs::VirtualFileSystem`. Los documentos oficiales y los mods se
  montan en el mismo árbol; las monturas posteriores tienen prioridad.
- El **log** y el archivo de configuración son los dos únicos ficheros que se
  tocan fuera del VFS, y los gestiona la capa de aplicación.

## Errores

- Los errores de programación usan `SIM_ASSERT` (`core/Assert.hpp`).
- Los errores recuperables se devuelven como `bool`, `std::optional` o
  estructuras de resultado (por ejemplo `ModReport`). Un mod inválido **nunca**
  aborta el proceso: se deshabilita con un mensaje claro.
- Las excepciones (`std::runtime_error`) se reservan a fallos de arranque
  irrecuperables (no se puede crear el dispositivo Vulkan, ...).
