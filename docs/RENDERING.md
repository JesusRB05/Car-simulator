# Rendering

The renderer is our own and is built on **Vulkan**. It relies only on utility
libraries: `vulkan-headers`, `volk` (dynamic loading of the Vulkan functions)
and `vulkan-memory-allocator` (VMA). No third-party graphics engine is used.

The code is split into two modules:

- `src/rhi`: a thin layer ("RHI") that wraps Vulkan.
- `src/render`: the scene, the camera, the meshes and the shaders, built on top
  of the RHI.

## RHI (`src/rhi`)

| Class | Responsibility |
|---|---|
| `Device` | Vulkan instance, physical/logical device, queues, VMA, debug messenger |
| `Swapchain` | Presentation chain and image views |
| `Frame` | CPU/GPU synchronisation, command-list pool, fences and semaphores |
| `Buffer` / `Image` | Resources allocated with VMA |
| `ShaderModule` / `ShaderCache` (the latter in `render`) | SPIR-V modules |
| `DescriptorSetLayout` / `DescriptorPool` / `DescriptorSet` | Descriptors |
| `GraphicsPipeline` / `ComputePipeline` | Pipelines |
| `CommandList` | Command recording |
| `Conversions` | Translation of our enums to Vulkan types |

### Platform isolation

The RHI **does not know about SDL**. The application creates the window, asks the
platform for the `VkSurfaceKHR` surface and passes it to the `Device` through a
factory function (`DeviceConfig::createSurface`), which is called after creating
the instance.

### volk and VMA

`volk` resolves all the Vulkan functions at runtime, so the **Vulkan loader is
not linked** and the SDK is not needed. The `VmaAllocator` is configured with
dynamic functions: the only unit that compiles the VMA implementation is
`src/rhi/src/Vma.cpp`, and `Device::createAllocator` passes it volk's
`vkGetInstanceProcAddr` and `vkGetDeviceProcAddr`.

### Frames in flight

`Frame` keeps `N` slots (2 by default). Each slot has a command list, a fence and
two semaphores (`imageAvailable`, `renderFinished`). The loop is:

```
frame.begin()             // waits on the slot fence, acquires the image
  commandList = frame.commandList()
  ... record ...
frame.submitAndPresent()  // closes, submits, presents
```

If the swapchain becomes outdated (`out of date`/`suboptimal`), `begin()` or
`present()` return `false` and the application recreates the swapchain and
retries.

## Render (`src/render`)

- **`Camera`**: perspective, right-handed, with **-Z forward** and **+Y up**.
  It provides `view()`, `projection()` and `viewProjection()`.
- **`Mesh`**: a CPU mesh (`position/normal/uv/tangent/colour` vertices + indices)
  uploaded to immutable buffers. It includes `makeCube`, `makeSphere`,
  `makeCylinder` and `makeGroundPlane`.
- **`Scene`**: a flat list of `Drawable` (mesh + transform + `Material`), one
  camera and one directional light. The simulator fills it every frame from the
  interpolated snapshot.
- **`RenderGraph`**: a **linear** graph of passes. Each pass is registered with a
  name and a function, and they run in order. It exists to add and reorder passes
  without touching the frame loop.
- **`Renderer`**: owns the scene pipelines (opaque + transparent), the directional
  shadow map, the per-frame UBOs, the descriptors and the *depth target*. It
  exposes `ensureTargets`, `updateFrameUniforms` and `renderScene`.

### Scene pipeline

Two passes per frame, both with **dynamic rendering** (`vkCmdBeginRendering`;
**no** `VkRenderPass`/`VkFramebuffer`):

1. **Shadow pass** — a depth-only pass into a 2048² `D32Sfloat` map (one image per
   frame in flight, so a frame's shadow pass never races another frame's shading).
   Its view-projection is a reversed-Z **orthographic** box fitted around the
   point the camera looks at (`reversedZOrtho` in `Renderer.cpp`).
2. **Scene pass** — the colour attachment plus the scene `D32Sfloat` depth in
   **reversed-Z** (`depthCompare = GreaterEqual`, clear to `0.0f`), cleared every
   frame. Opaque drawables are drawn first (with alpha-test discard for masked
   materials); alpha-blended drawables are sorted back-to-front and drawn last
   with depth writes off.

- `TriangleList`, **no** culling: both faces are drawn so open and thin (glass)
  geometry reads correctly, and the shader flips the normal for back faces.
- One UBO per frame (binding 0) with `viewProjection`, the light's
  view-projection, the shadow matrix, camera position, light direction/colour and
  the hemispheric ambient. Set 1 is the shadow map (a sampled image + a sampler).
- Per object, **push constants** with the model matrix and the material factors.

#### Materials

A `render::Material` carries the glTF factors (base colour, emissive colour and
strength, metallic, roughness, clearcoat, alpha cutoff, and the `doubleSided` /
`alphaBlend` / `alphaMask` flags) plus two optional maps: the **base colour**
(sRGB) and the **emissive** (sRGB). Everything else the two shipped assets use is
still a factor: normal, metallic-roughness and occlusion maps are read by the
loader but not sampled yet. The fragment shader is **metallic-roughness** (GGX
distribution, Smith-Schlick geometry, Schlick Fresnel) with a clearcoat
approximation, lit by one directional sun, plus a hemispheric ambient derived
from the sky colour's luminance, an ACES tone map and an exposure scalar from
`Scene::exposure()`. Emissive is game-driven (brake and rain lights). The sun's
shadow lookup compares against the reversed-Z stored depth with a 3×3 PCF kernel
and a small normal offset.

Textures are uploaded by `render::Texture`: the images the VFS decodes are
expanded to RGBA8, box-filtered down to a full mip chain (2×2 box per level) and
sampled with anisotropic-enabled trilinear filtering. Colour maps get an sRGB
view; the material factors stay linear. Missing maps resolve to a 1×1 white
(base colour) / black (emissive) stand-in, so the shader always samples. The
material's two views are cached in a descriptor set of their own (set 2), so the
renderer does not rebuild one per drawable.

Skin ("livery") maps are not part of the glTF: `render::loadModel` accepts a list
of `MaterialOverride` (material, channel, image) and decodes them through the
same image path, so a skin costs one texture and no shader work.

### Circuits from glTF

A modelled circuit (for example `base.core/sierra_verde`) is not drawn through
`render::loadModel`: `drive/TrackModel` imports it device-free, because the same
triangles have to serve two masters.

- **Drawables**: every node with a mesh is transformed to world space and split
  on a 48 m grid over XZ when a primitive has more than 8192 triangles. The
  terrain (105k triangles), the armco (88k) and the tyre walls (58k) become some
  130, 90 and 60 drawables with their own bounds, so a per-drawable frustum test
  (next step) can drop the 90 % of the circuit that is behind the camera. The
  tiles share the glTF's materials, and `render::uploadMaterials` uploads one
  texture per image, so the whole circuit costs 28 textures, not one per tile.
- **Drivable surface**: `TrackSurfaceQuery` builds a `physics::TriangleMesh` BVH
  from the collision nodes' triangles whose material names a surface, plus a
  per-triangle friction vector. A vertical ray gives the wheel the height, the
  normal and the friction in one query. Materials without a surface (roofs,
  glass, trees) never enter it, which keeps a downward ray from landing on the
  footbridge above the road.
- **Start pose**: the `track.json` layout, not the mesh, says where the line and
  the grid are.

The BVH slab test is written so a ray whose origin lies exactly on a node's
face is not pruned: the road is generated as two halves that meet on the
centreline, and a ray at `x = 0` used to fall through that seam.

#### Level of detail and culling

Every drawable carries its own world bounds and is frustum-culled per frame, so
the part of the circuit behind the camera costs nothing. The large tiles also
carry a LOD chain built once at import time with `meshoptimizer`
(`drive/MeshLods`):

- A tile with more than 512 triangles gets two simplified levels at 50 % and
  25 % of its triangles (`targetError = 0.01`, `minSaving = 0.9`).
- The level is chosen by the distance from the camera to the tile's bounds
  centre: level 0 up to 140 m, level 1 up to 380 m and level 2 beyond
  (`kLodDistances` in `drive/TrackVisual.cpp`; the choice itself is
  `render::selectLodLevel`, device-free and unit-tested).
- **Borders are locked by hand**, because `meshopt_SimplifyLockBorder` is not
  enough for glTF: a texture seam is classified as `Kind_Seam` and may collapse.
  `MeshLods` welds vertices by exact position bits, finds the edges that appear
  once and passes those vertices as locked. A tile that is mostly border (an
  armco rail, a painted line) gets no chain at all, since simplifying it would
  open gaps against its neighbour.
- The LOD meshes share the level-0 vertex buffer and only add an index buffer,
  so the chain costs indices, not vertices; the shadow pass selects a level with
  the same rule.

On the reference machine (AMD Radeon integrated graphics, 1920x1080) Sierra
Verde with three cars on the grid renders at **309 fps (3.2 ms/frame)**; the
60 fps target has a wide margin, so the circuit keeps its full detail.

The offscreen target is linear `R8G8B8A8_UNORM`; the window swapchain is
`B8G8R8A8_SRGB`, so the hardware applies the sRGB encode there and the shaders
always output **linear** colour.

### Depth precision (reversed-Z)

The projection is right-handed with reverse depth (near → 1, far → 0). For
`near = 0.1`, `far = 1000` and `D32Sfloat`, the buffer resolution at a distance
`z` is

$$
\delta z \approx \frac{2^{-24}}{\frac{F\,N}{(F-N)}\,\frac{1}{z^2}} \approx 0.2\;\text{mm}\ \text{at } z = 18\ \text{m}.
$$

Precision improves with distance (which is the reason for using reversed-Z), but
there is a practical problem on support planes: a huge ground quad standing at
the same height as a supported slab produces **depth ties** flush in grazing
views, and the ground can win the *depth test*.

Rule for scene geometry: **every surface resting on another is separated by a
minimum take-off of 1–2 mm** (≈10 depth units at 20 m, never a tie). The `checker`
scene applies this rule: its slabs are 10 mm plates floating 2 mm above the
ground. Do not use *depth bias* unless a pass explicitly asks for it: it breaks
the similarity with the geometry and depends on the driver.

### `FrameUniforms` / `ObjectPushConstants` format

Single matrix convention: **GLM column-major on the CPU and `column_major` in
Slang**, multiplying with `mul(M, v)` (column vector). `glm::mat4` is copied to
the GPU **verbatim** with `writeMatrix()` (`render/Uniforms.hpp`); there are no
transpositions anywhere. The layout is std140: every member aligned to 16 bytes,
and a matrix occupies 4 contiguous 16-byte columns.

```
FrameUniforms:        viewProjection, lightViewProjection, shadowMatrix (mat4),
                      cameraPosition, lightDirection, lightColor,
                      ambientSky (rgb ambient + a exposure), ambientGround
ObjectPushConstants:  model (mat4), baseColor (rgb + a opacity),
                      emissive (rgb + a strength),
                      params (metallic, roughness, clearcoat, alpha cutoff)
```

## Shaders (Slang)

`src/render/shaders/mesh.slang` defines the two stages of the scene pipeline:

- `vertexMain`: transforms to world space, then to clip space, and rotates the
  normal.
- `fragmentMain`: physically-based **metallic-roughness** shading of one
  directional sun (GGX + Smith-Schlick + Schlick Fresnel, an approximate
  clearcoat lobe and a hemispheric ambient), an ACES tone map and the exposure
  scalar. The surface normal is interpolated (smooth); back faces flip it.

`src/render/shaders/shadow.slang` is the depth-only shadow pass; it reuses the
frame uniform buffer (set 0) and the same push constants, so the two pipelines
share the vertex layout.

They are compiled to SPIR-V during the build by the `sim_slang_shader` function,
which generates `shaders/mesh.vertex.spv`, `shaders/mesh.fragment.spv`,
`shaders/shadow.vertex.spv` and `shaders/shadow.fragment.spv` and copies them
next to the executable.

### Compilation with `-emit-spirv-via-glsl`

`sim_slang_shader` compiles **through GLSL** (`slangc -emit-spirv-via-glsl`),
not with the direct Slang-to-SPIR-V path. The direct path marks the module with
`Source Language = Slang (11)`, which software drivers (SwiftShader/lavapipe)
reject with `Invalid source language operand: 11` and then crash. The GLSL path
emits `Source Language = GLSL`, accepted by all drivers.

**It will be revisited** when we need a Slang feature that does not go through
GLSL (for example *ray tracing* or `[shader("raygeneration")]` in the graphics
part); at that point we will evaluate returning to the direct path and requiring
a real driver. On a **real GPU the behaviour is identical** with either path: the
resulting SPIR-V is functionally equivalent, only the origin metadata differs.

## Rendering without a GPU (headless)

The `sim` executable supports an *offscreen* mode intended for automatic
verification (CI without a GPU, image regression tests):

```
sim --headless --frames N [--scene NAME] [--screenshot path.png]
    [--width W] [--height H] [--resize N] [--validation]
    [--verify-golden|--update-golden] [--golden-dir DIR] [--semantic]
    [--contact-sheet path.png] [--debug-draw]
    [--camera chase|cockpit|orbit|side] [--time SECONDS] [--yaw RAD] [--pitch RAD]
```

- **No window or swapchain**: it creates an `R8G8B8A8Unorm` colour image and a
  *depth target* and renders offscreen; it does not link with SDL
  (`SDL_GetBasePath` before `SDL_Init` corrupts the heap, so the executable
  directory is derived from `argv[0]`).
- **`--resize N`**: recreates the *targets* at half size on frame `N` to exercise
  the recreation path (the offscreen equivalent of a window resize).
- **`--screenshot`**: writes the last frame as a PNG (stb) through an
  image→buffer copy.
- **`--validation`**: enables the layers and **returns exit code 2** if any
  validation message was emitted; code 1 if Vulkan initialisation fails.
- **`--verify-golden` / `--update-golden`**: compares or regenerates the
  reference in `--golden-dir`. `--semantic` adds the projected-pixel check for
  each scene probe. Verification **is skipped (code 77)** if the reference does
  not exist yet (not approved); `update_golden` writes the **candidates** to
  `screenshots/golden_candidates/`, which only move to `tests/golden/<driver>/`
  after human approval. See `tests/golden/README.md`.
- **`--debug-draw`**: overlays the ground grid and the world axes through the
  instanced *debug draw* (`render::DebugDraw`); its reference is
  `tests/golden/<driver>/grid_debug.png`.
- **Drive capture** (`--scene drive`): `--camera chase|cockpit|orbit|side`
  selects the view, `--yaw`/`--pitch` set the orbit angles, and `--time S`
  drives the car with the built-in autopilot for `S` seconds before the capture
  (`0` leaves it parked on the start/finish line). `tools\capture_readme_shots.bat`
  uses these to regenerate the README screenshots.
- **Benchmark** (`--benchmark --frames N [--grid-cars C]`): renders one warm-up
  frame, then `N` frames each submitted and waited on, and prints
  `[headless] benchmark '<scene>': N frame(s) in X s, Y ms/frame, Z fps (WxH)`.
  `--grid-cars C` fills the starting grid with `C` parked cars built from the
  real car model, which is how the circuit is measured with traffic.
- **Physics scenes** (`drop`, `oscillator`, `pendulum`, `double_pendulum`,
  `chain`, `dzhanibekov`, `gyro`, `masses`): they build a real `PhysicsWorld` and
  integrate it for **1.5 s with the deterministic fixed step** before capturing,
  with the camera predefined per scene. They always carry the overlay active
  (grid + axes + a scene-specific line: analytic trajectory, ball joints/springs
  or body axes), so the reference anchors both the physical state and its
  visualisation. They carry no semantic probes (the geometries move).
- **Screen orientation**: all conversion from world `+Y` to screen lives in a
  single place, the negative-height viewport of `CommandList::setViewport` (see
  `docs/CONVENTIONS.md`); the `checkSemantics` probes report the result. The
  `orientation` test scene adds a horizon check (`checkOrientationHorizon`) that
  always runs, independent of the golden images, so an orientation failure cannot
  go unnoticed.
- The *runner* is in `apps/sim/src/HeadlessRunner.{hpp,cpp}`.

The `gpu` tests (`ctest -L gpu`) compare each scene against references in
`tests/golden/<driver>/` (see `docs/BUILDING.md`) and a contact sheet
`screenshots/contact_sheet.png` (committed by hand) gathers all the scenes, with
the physics ones captured after their 1.5 s settling speed.

## Instanced debug draw

`render::DebugDraw` (`src/render/{include/render/DebugDraw.hpp,src/DebugDraw.cpp}`)
draws line segments in a single instanced call: each instance supplies its two
endpoints and its colour (`debug.slang`), with no per-line vertex buffer. It is
drawn **without a depth test** (`depthTest=false`) as an overlay, so it serves
for grids, axes and gizmos, not as occluded geometry. In the window it is toggled
with `G` (or the checkboxes in the *Debug draw* panel); in headless mode with
`--debug-draw`. In the **physics scenes** the overlay is active by default in both
modes (grid + axes + the scene's own lines), so the state of the world is visible
without extra steps.

## ImGui

The debug interface uses ImGui with the SDL3 and Vulkan backends (the project's
own, compiled in `sim::imgui_backends`). They are compiled with
`IMGUI_IMPL_VULKAN_USE_VOLK` so they use volk instead of the Vulkan loader.
ImGui creates its own *descriptor pool* and uses the same dynamic rendering as
the scene. The application passes it each SDL event through a hook, and calls
`ImGui_ImplVulkan_SetMinImageCount` when recreating the swapchain.

`implot` is available for plots (for example, telemetry).

See [UI.md](UI.md) for the windowed app's menus, panels and keyboard shortcuts.

## Conventions

- World in metres; **+Y up; -Z forward**.
- Back-face culling: triangles are wound **counter-clockwise (CCW)** seen from
  outside.
- All content (player, scene) is converted to `float` before touching the GPU.
