# Renderizado

El renderizador es propio y está construido sobre **Vulkan**. Se apoya solo en
bibliotecas de utilidad: `vulkan-headers`, `volk` (carga dinámica de las
funciones de Vulkan) y `vulkan-memory-allocator` (VMA). No se usa ningún motor
gráfico de terceros.

El código se divide en dos módulos:

- `src/rhi`: capa fina ("RHI") que encapsula Vulkan.
- `src/render`: la escena, la cámara, las mallas y los shaders, construidos
  sobre el RHI.

## RHI (`src/rhi`)

| Clase | Responsabilidad |
|---|---|
| `Device` | Instancia Vulkan, dispositivo físico/lógico, colas, VMA, mensajero de depuración |
| `Swapchain` | Cadena de presentación e image views |
| `Frame` | Sincronización CPU/GPU, pool de listas de comandos, fences y semáforos |
| `Buffer` / `Image` | Recursos asignados con VMA |
| `ShaderModule` / `ShaderCache` (este último en `render`) | Módulos SPIR-V |
| `DescriptorSetLayout` / `DescriptorPool` / `DescriptorSet` | Descriptores |
| `GraphicsPipeline` / `ComputePipeline` | Pipelines |
| `CommandList` | Grabación de comandos |
| `Conversions` | Traducción de los enums propios a tipos de Vulkan |

### Aislamiento de la plataforma

El RHI **no conoce SDL**. La aplicación crea la ventana, pide la superficie
`VkSurfaceKHR` a la plataforma y se la pasa al `Device` a través de una función
de fábrica (`DeviceConfig::createSurface`), que se invoca después de crear la
instancia.

### volk y VMA

`volk` resuelve todas las funciones de Vulkan en tiempo de ejecución, por lo
que **no se enlaza el cargador de Vulkan** ni hace falta el SDK. El
`VmaAllocator` se configura con funciones dinámicas: la única unidad que compila
la implementación de VMA es `src/rhi/src/Vma.cpp`, y `Device::createAllocator`
le pasa `vkGetInstanceProcAddr` y `vkGetDeviceProcAddr` de volk.

### Frames en vuelo

`Frame` mantiene `N` slots (2 por defecto). Cada slot tiene una lista de
comandos, un fence y dos semáforos (`imageAvailable`, `renderFinished`). El
bucle es:

```
frame.begin()             // espera el fence del slot, adquiere la imagen
  commandList = frame.commandList()
  ... grabar ...
frame.submitAndPresent()  // cierra, envía, presenta
```

Si la swapchain queda desactualizada (`out of date`/`suboptimal`), `begin()` o
`present()` devuelven `false` y la aplicación recrea la swapchain y reintenta.

## Render (`src/render`)

- **`Camera`**: perspectiva, diestra, con **-Z adelante** y **+Y arriba**.
  Ofrece `view()`, `projection()` y `viewProjection()`.
- **`Mesh`**: malla de CPU (vértices `posición/normal/uv` + índices) subida a
  búferes inmutables. Incluye `makeCube` y `makeGroundPlane`.
- **`Scene`**: lista plana de `Drawable` (malla + transformación + color base),
  una cámara y una luz direccional. El simulador la rellena cada frame a partir
  del snapshot interpolado.
- **`RenderGraph`**: grafo **lineal** de pasadas. Cada pasada se registra con un
  nombre y una función, y se ejecutan en orden. Existe para añadir y reordenar
  pasadas sin tocar el bucle de frame.
- **`Renderer`**: posee el pipeline de escena, los UBO por frame, los
  descriptores y el *depth target*. Expone `ensureTargets`, `updateFrameUniforms`
  y `renderScene`.

### Pipeline de escena

- Una sola pasada con **renderizado dinámico** (`vkCmdBeginRendering`;
  **sin** `VkRenderPass`/`VkFramebuffer`).
- Profundidad `D32Sfloat` en **reversed-Z** (`depthCompare = GreaterEqual`, clear a
  `0.0f`), limpiada cada frame.
- `TriangleList`, culling de caras traseras.
- Un UBO por frame (binding 0) con `viewProjection`, posición de cámara, dirección y
  color de la luz.
- Por objeto, **push constants** con la matriz de modelo y el color base.

### Precisión de profundidad (reversed-Z)

La proyección es diestra con profundidad inversa (cerca → 1, lejos → 0). Para
`near = 0,1`, `far = 1000` y `D32Sfloat`, la resolución del buffer a una distancia
`z` es

$$
\delta z \approx \frac{2^{-24}}{\frac{F\,N}{(F-N)}\,\frac{1}{z^2}} \approx 0{,}2\;\text{mm}\ \text{a } z = 18\ \text{m}.
$$

La precisión mejora con la distancia (es el motivo de usar reversed-Z), pero en
los planos de apoyo hay un problema práctico: un quad de suelo enorme en pie a la
misma altura de una losa apoyada produce **empates de profundidad** al ras en
vistas rasantes, y el suelo puede ganar el *depth test*.

Regla para geometría de escena: **toda superficie apoyada en otra se separa con
un despegue mínimo de 1–2 mm** (≈10 unidades de profundidad a 20 m, nunca un
empate). La escena `checker` aplica esta regla: sus losas son planchas de 10 mm
flotando 2 mm sobre el suelo. No usar *depth bias* salvo que una pasada lo pida
explícitamente: rompe la semejanza con la geometría y depende del driver.

### Formato de `FrameUniforms` / `ObjectPushConstants`

Convención única de matrices: **GLM column-major en CPU y `column_major` en
Slang**, multiplicando con `mul(M, v)` (vector columna). `glm::mat4` se copia a
la GPU **verbatim** con `writeMatrix()` (`render/Uniforms.hpp`); no hay
transposiciones en ningún sitio. El layout es std140: cada miembro alineado a 16
bytes, y una matriz ocupa 4 columnas de 16 bytes contiguas.

```
FrameUniforms:        viewProjection (mat4), cameraPosition, lightDirection, lightColor
ObjectPushConstants:  model (mat4),           baseColor
```

## Shaders (Slang)

`src/render/shaders/mesh.slang` define las dos etapas del pipeline de escena:

- `vertexMain`: transforma a espacio de mundo, luego a clip, y rota la normal.
- `fragmentMain`: Lambert sencillo,
  `color = baseColor.rgb * (0.18 + max(0, n·l) * lightColor.rgb)`.

Se compilan a SPIR-V durante el build con la función `sim_slang_shader`, que
genera `shaders/mesh.vertex.spv` y `shaders/mesh.fragment.spv` y los copia
junto al ejecutable.

### Compilación con `-emit-spirv-via-glsl`

`sim_slang_shader` compila **a través de GLSL** (`slangc -emit-spirv-via-glsl`),
no con la ruta directa de Slang a SPIR-V. La ruta directa marca el módulo con
`Source Language = Slang (11)`, que los drivers software (SwiftShader/lavapipe)
rechazan con `Invalid source language operand: 11` y luego provocan un fallo.
La ruta GLSL emite `Source Language = GLSL`, aceptada por todos los drivers.

**Se revisará** cuando necesitemos una funcionalidad de Slang que no pase por
GLSL (p. ej. *ray tracing* o `[shader("raygeneration")]` en la parte gráfica);
en ese momento se evaluará volver a la ruta directa y exigir un driver real.
En **GPU real el comportamiento es idéntico** con cualquiera de las dos rutas:
el SPIR-V resultante es funcionalmente equivalente, solo difiere la metainformación
de origen.

## Renderizado sin GPU (headless)

El ejecutable `sim` admite un modo *offscreen* pensado para verificación
automática (CI sin GPU, tests de regresión de imagen):

```
sim --headless --frames N [--scene NAME] [--screenshot ruta.png]
    [--width W] [--height H] [--resize N] [--validation]
    [--verify-golden|--update-golden] [--golden-dir DIR] [--semantic]
    [--contact-sheet ruta.png] [--debug-draw]
```

- **Sin ventana ni swapchain**: crea una imagen de color `R8G8B8A8Unorm` y un
  *depth target* y renderiza fuera de pantalla; no enlaza con SDL
  (`SDL_GetBasePath` antes de `SDL_Init` corrompe el heap, por eso el directorio
  del ejecutable se deriva de `argv[0]`).
- **`--resize N`**: recrea los *targets* a mitad de tamaño en el frame `N` para
  probar la ruta de recreación (equivalente offscreen del reescalado de ventana).
- **`--screenshot`**: escribe el último frame como PNG (stb) mediante copia
  imagen→búfer.
- **`--validation`**: activa las capas y **devuelve código 2** si se emitió algún
  mensaje de validación; código 1 si falla la inicialización de Vulkan.
- **`--verify-golden` / `--update-golden`**: compara o regenera la referencia de
  `--golden-dir` (por defecto `tests/golden/`). `--semantic` añade la
  comprobación de píxel proyectado de cada sonda de escena.
- **`--debug-draw`**: superpone la rejilla de suelo y los ejes del mundo mediante
  el *debug draw* instanciado (`render::DebugDraw`); su referencia es
  `tests/golden/grid_debug.png`.
- El *runner* está en `apps/sim/src/HeadlessRunner.{hpp,cpp}`.

Los tests `gpu` (etiqueta `ctest -L gpu`) comparan cada escena con referencias en
`tests/golden/` (ver `docs/CONSTRUCCION.md`) y una hoja de contacto
`screenshots/contact_sheet.png` reúne todas las escenas.

## Debug draw instanciado

`render::DebugDraw` (`src/render/{include/render/DebugDraw.hpp,src/DebugDraw.cpp}`)
dibuja segmentos de línea en una sola llamada instanciada: cada instancia aporta
sus dos extremos y su color (`debug.slang`), sin búfer de vértices por línea. Se
dibuja **sin test de profundidad** (`depthTest=false`) como superposición, por lo
que sirve para rejillas, ejes y gizmos, no como geometría ocluida. En la ventana
se alterna con `G` (o la casilla «Debug grid» del panel *View*); en headless con
`--debug-draw`.

## ImGui

La interfaz de depuración usa ImGui con los backends de SDL3 y Vulkan (propios
del proyecto, compilados en `sim::imgui_backends`). Se compilan con
`IMGUI_IMPL_VULKAN_USE_VOLK` para que usen volk en lugar del cargador de Vulkan.
ImGui crea su propio *descriptor pool* y usa el mismo renderizado dinámico que
la escena. La aplicación le pasa cada evento SDL mediante un enganche, y llama
`ImGui_ImplVulkan_SetMinImageCount` al recrear la swapchain.

`implot` está disponible para gráficas (por ejemplo, la telemetría).

## Convenciones

- Mundo en metros; **+Y arriba; -Z adelante**.
- Culling de caras traseras: los triángulos van en **sentido antihorario (CCW)**
  vistos desde fuera.
- Todo el contenido (jugador, escena) se pasa a `float` antes de tocar la GPU.
