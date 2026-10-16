# Native plugins

When a mod needs native performance or access to C/C++ code, it can include a
**plugin**: a shared library (`.dll`, `.so`, `.dylib`) that implements a stable
C ABI.

## ABI

The ABI is defined in `src/plugin_api/include/plugin_api/plugin_api.h`, which
is **plain C** (no C++ constructs), so a plugin can be written in C, C++ or any
language able to produce a shared library with a C ABI. Current version: **1**.

A plugin exports exactly two symbols:

```c
uint32_t sim_plugin_abi_version(void);
int      sim_plugin_entry(uint32_t host_abi_version,
                          const sim_plugin_api* api,
                          sim_plugin_host* out_host);
```

1. The host loads the library and reads `sim_plugin_abi_version()`.
2. It compares the value with `SIM_PLUGIN_ABI_VERSION`. If they do not match,
   the plugin is rejected.
3. It calls `sim_plugin_entry()`, passing the **host ABI version** (in case the
   plugin wants to reject a host it does not understand) and the
   `sim_plugin_api` service block.
4. The plugin fills `sim_plugin_host` with its *callbacks* and returns `0` on
   success, or a non-zero code on error.

### Services offered by the host (`sim_plugin_api`)

| Field | Description |
|---|---|
| `abi_version`, `struct_size` | Version and size of the structure |
| `user_data` | Opaque host context; must be passed back on every call |
| `game_version` | Game version |
| `log(user_data, level, message)` | Writes to the plugin log |
| `vfs_read(user_data, path, buffer, size)` | Reads from the VFS; returns the bytes written or `-1` |
| `plugin_name(user_data)` | Plugin name |

### Callbacks returned by the plugin (`sim_plugin_host`)

| Field | Description |
|---|---|
| `user_data` | Plugin context; passed back in the callbacks |
| `on_update(user_data, dt)` | Once per frame, with the delta in seconds |
| `on_shutdown(user_data)` | Once when unloading; may be `NULL` |

## Compatibility between versions

Structures **only grow by appending fields at the end**. `struct_size` lets
both sides ignore fields they do not know: an old plugin can load on a new host
(and vice versa) as long as this rule is respected. `abi_version` is only
incremented for incompatible changes.

## Example

`plugins/example_native/` is a reference C plugin that demonstrates the ABI end
to end: it reports its ABI version, receives the API, writes to the log, reads
`/README.txt` from the VFS and counts frames in `on_update`. It is built as a
module (`add_library(... MODULE ...)`) without the `lib` prefix, and its output
goes to `bin/plugins/`.

## Loading at run time

Native plugins are **opt-in and off by default**. Dropping a library into
`plugins/` never runs it: the loader only loads a file when the user has both
switched plugins on and listed that file, in `config.toml`:

```toml
[plugins]
enabled = true
allow = ["example_native.dll"]   # file name, or the bare stem
```

`PluginLoader::loadDirectory(directory, options)` walks the directory for
shared libraries and **fingerprints every one of them (SHA-256)** before doing
anything else, so the log always records which binary is about to be trusted:

- With `enabled = false` nothing is loaded and every library is logged as
  skipped, with its hash. The Mods panel in the UI shows the same warning.
- With `enabled = true`, only the libraries named in `allow` are loaded; the
  rest are logged and shown as skipped, with their hashes.
- A library that is allowed but fails (does not open, is missing symbols, has an
  incompatible ABI or `sim_plugin_entry` returns an error) is recorded with its
  error message and **does not prevent** the rest from loading.

`update(dt)` calls every loaded `on_update`; `unloadAll()` calls the
`on_shutdown` callbacks and unloads the libraries.

On Windows the library is loaded with `LoadLibraryExW` and an explicit search
mode: `SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS)` narrows the
process-wide default, and `LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR` makes the plugin's
own directory the first place its dependencies are looked up. The current
directory and `PATH` are never searched for a plugin or its dependencies, which
closes the classic DLL-hijacking vector. Elsewhere the loader uses
`SDL_LoadObject` / `SDL_LoadFunction` / `SDL_UnloadObject`.

Each plugin has its own logger (`core::Log::forPlugin(name)`), so its messages
are tagged with the plugin name.

## Writing a plugin

1. Include `plugin_api/plugin_api.h`.
2. Export `sim_plugin_abi_version` and `sim_plugin_entry` with
   `SIM_PLUGIN_EXPORT`.
3. Check `host_abi_version` and `api->struct_size` before using the API.
4. Fill `out_host` and return `0`.
5. Build it as a shared library (module) and place it in the `plugins/` folder
   next to the executable.
6. Switch plugins on and allowlist the file in `config.toml` (see above). Note
   the SHA-256 the loader logs, so you can confirm you are running the binary
   you built.
