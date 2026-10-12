# Plugins nativos

Cuando un mod necesita rendimiento nativo o acceso a código C/C++, puede
incluir un **plugin**: una biblioteca compartida (`.dll`, `.so`, `.dylib`) que
implementa una ABI C estable.

## ABI

La ABI se define en `src/plugin_api/include/plugin_api/plugin_api.h`, que es
**C puro** (sin construcciones de C++), de modo que un plugin puede escribirse
en C, C++ o cualquier lenguaje capaz de producir una biblioteca compartida con
ABI C. Versión actual: **1**.

Un plugin exporta exactamente dos símbolos:

```c
uint32_t sim_plugin_abi_version(void);
int      sim_plugin_entry(uint32_t host_abi_version,
                          const sim_plugin_api* api,
                          sim_plugin_host* out_host);
```

1. El host carga la biblioteca y lee `sim_plugin_abi_version()`.
2. Compara el valor con `SIM_PLUGIN_ABI_VERSION`. Si no coincide, rechaza el
   plugin.
3. Llama a `sim_plugin_entry()` pasándole la **versión de la ABI del host** (por
   si el plugin quiere rechazar un host que no entiende) y el bloque de
   servicios `sim_plugin_api`.
4. El plugin rellena `sim_plugin_host` con sus *callbacks* y devuelve `0` si
   todo va bien, o un código distinto de cero en caso de error.

### Servicios que el host ofrece (`sim_plugin_api`)

| Campo | Descripción |
|---|---|
| `abi_version`, `struct_size` | Versión y tamaño de la estructura |
| `user_data` | Contexto opaco del host; hay que devolverlo en cada llamada |
| `game_version` | Versión del juego |
| `log(user_data, nivel, mensaje)` | Escribe en el log del plugin |
| `vfs_read(user_data, ruta, buffer, tamaño)` | Lee del VFS; devuelve los bytes escritos o `-1` |
| `plugin_name(user_data)` | Nombre del plugin |

### Callbacks que el plugin devuelve (`sim_plugin_host`)

| Campo | Descripción |
|---|---|
| `user_data` | Contexto del plugin; se devuelve en los callbacks |
| `on_update(user_data, dt)` | Una vez por frame, con el delta en segundos |
| `on_shutdown(user_data)` | Una vez al descargar; puede ser `NULL` |

## Compatibilidad entre versiones

Las estructuras **solo crecen añadiendo campos al final**. `struct_size` permite
que ambos lados ignoren los campos que no conocen: un plugin viejo puede cargar
sobre un host nuevo (y viceversa) mientras se respete esta regla. La
`abi_version` solo se incrementa ante cambios incompatibles.

## Ejemplo

`plugins/example_native/` es un plugin de referencia en C que demuestra la ABI
de extremo a extremo: informa de su versión ABI, recibe la API, escribe en el
log, lee `/README.txt` del VFS y cuenta frames en `on_update`. Se compila como
módulo (`add_library(... MODULE ...)`) sin prefijo `lib`, y su salida va a
`bin/plugins/`.

## Carga en tiempo de ejecución

`PluginLoader` carga una biblioteca o un directorio entero (recorriendo los
ficheros con extensión de biblioteca compartida). Usa `SDL_LoadObject` /
`SDL_LoadFunction` / `SDL_UnloadObject` para el trabajo multiplataforma.

- Un plugin que falla (no abre, le faltan símbolos, ABI incompatible o
  `sim_plugin_entry` devuelve error) se registra con su mensaje de error y **no
  impide** cargar el resto.
- `update(dt)` llama a todos los `on_update` cargados.
- `unloadAll()` llama a los `on_shutdown` y descarga las bibliotecas.

Cada plugin tiene su propio logger (`core::Log::forPlugin(nombre)`), de modo que
sus mensajes quedan etiquetados con el nombre del plugin.

## Escribir un plugin

1. Incluye `plugin_api/plugin_api.h`.
2. Exporta `sim_plugin_abi_version` y `sim_plugin_entry` con
   `SIM_PLUGIN_EXPORT`.
3. Comprueba `host_abi_version` y `api->struct_size` antes de usar la API.
4. Rellena `out_host` y devuelve `0`.
5. Compila como biblioteca compartida (módulo) y colócala en la carpeta
   `plugins/` junto al ejecutable.
