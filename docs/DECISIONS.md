# Decisiones de diseño

Registro de las decisiones técnicas más importantes y por qué se tomaron.

## Motor propio: nada de motores de física ni de render de terceros

El proyecto implementa su propio motor de física y su propio renderizador
Vulkan. Está **prohibido** usar motores como Jolt, Bullet o PhysX para la
física, o bgfx, Diligent o Filament para el render. El objetivo es que el
proyecto sea plenamente comprensible y modificable por la comunidad. Sí se usan
bibliotecas de utilidad (volk, VMA, glm, ImGui, ...).

## Unidades y convenciones

- Todo el cálculo interno va en **unidades del SI** (metros, kilogramos,
  segundos, radianes).
- Sistema **diestro**, **+Y arriba**, **-Z adelante**.
- Las posiciones del mundo se guardan en **`double`**; la física local y lo que
  se envía al render van en **`float`**. Así se evita la pérdida de precisión en
  mapas grandes sin pagar `double` en toda la geometría.

## Física determinista en un hilo aparte

- **Paso fijo** (por defecto `1/1000` s). La física **no** usa el `dt` del
  frame.
- Corre en su **propio hilo**. El render **interpola** entre dos snapshots.
- La comunicación es un **anillo sin bloqueos** (`SnapshotRing`, SPSC con 128
  slots): no hay mutex en el camino crítico.
- Determinismo estricto: MSVC con `/fp:precise`, GCC/Clang con
  `-ffp-contract=off`. **Nunca** `-ffast-math`, que rompería la reproducibilidad
  necesaria para replays y para depurar.

## Aislamiento del RHI respecto a la plataforma

El RHI (`src/rhi`) **no conoce SDL**. La plataforma ofrece una fábrica que crea
la `VkSurfaceKHR` y el `Device` la pide por `DeviceConfig`. Así el render es
independiente de la ventana y podría reutilizarse con otra plataforma.

## volk + VMA en lugar del cargador de Vulkan

Se usa **volk** para cargar dinámicamente las funciones de Vulkan, de modo que
**no hace falta el SDK de Vulkan** ni enlazar el cargador. VMA se compila con
`VMA_IMPLEMENTATION` en una sola unidad (`src/rhi/src/Vma.cpp`) y se enlaza con
funciones dinámicas (`vkGetInstanceProcAddr`/`vkGetDeviceProcAddr`).

## Renderizado dinámico (sin render passes)

Las pasadas usan `vkCmdBeginRendering` en vez de `VkRenderPass` +
`VkFramebuffer`. Es más simple, evita recombinaciones de pases y es el camino
moderno de Vulkan 1.3.

## VFS con PhysicsFS

El acceso a contenido pasa siempre por un sistema de ficheros virtual
(PhysicsFS). El contenido base y cada mod se montan **todos en `/`**; la
prioridad de montaje decide quién gana. Esto hace que los mods puedan
sobrescribir ficheros sin ninguna lógica especial en el motor.

## Scripting aislado (LuaJIT + sol2)

Los scripts corren en un **sandbox** (sin `io`, `os`, `package`, `debug`, `ffi`;
sin `load`/`require`/`dofile`; solo texto, nunca bytecode) y con un **límite de
instrucciones**. Un script travieso o con un bucle infinito no puede tumbar el
simulador.

## ABI C para plugins

La interfaz de plugins es **C puro** y versionada (`struct_size` + versión). Las
estructuras solo crecen por el final, de modo que un plugin compilado contra una
versión pueda seguir funcionando. Esto permite plugins en cualquier lenguaje que
genere ABI C.

## Licencia: GPLv3 con excepción de enlazado

El motor es **GPL-3.0-or-later** con una **excepción de enlazado**, para que los
plugins y mods no tengan que ser GPL solo por enlazar contra el motor. El
contenido oficial es **CC BY-SA 4.0**. Sin marcas ni coches reales.

## CMake moderno con presets y vcpkg

Configuración con **CMake ≥ 3.25**, **vcpkg manifest** y **CMakePresets**.
Facilita reproducir el entorno en Windows (MSVC) y Linux (GCC/Clang) sin pasos
manuales. Slang no está en vcpkg, así que se descarga como binario precompilado
(FetchContent) y se puede sobrescribir con `SIM_SLANGC`.

## Slang para shaders

Los shaders se escriben en **Slang** y se compilan a SPIR-V en tiempo de build.
Slang es un lenguaje moderno con buena ergonomía para paso de estructuras entre
CPU y GPU.
