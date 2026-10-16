# Mod system

The engine is designed to be moddable. Mods **do not change the binary**: they
are mounted on the virtual file system (VFS) and can optionally add Lua scripts
or native plugins.

## Where a mod lives

```
mods/
  my_mod/
    mod.toml            <- required manifest
    scripts/main.lua    <- optional (entry_script)
    cars/<id>/car.toml  <- optional; adds a car
    tracks/<id>/track.toml  <- optional; adds a circuit
    ...                 <- any other content (textures, meshes, data)
```

A mod can add a **car** or a **track** with a descriptor only: the catalogs scan
the VFS at start-up and list everything they find, so `cars/<id>/car.toml` and
`tracks/<id>/track.toml` show up next to the official ones. See
[CONTENT.md](CONTENT.md) for the schema.

A mod can be a **directory** or a **`.zip`**. The manager scans `mods/`, and
every entry is a mod candidate. The official content (`content/base`) is, as
far as the system is concerned, just another mod (the "base" one), always
present and with the lowest priority.

`ModManager`:

1. Discovers candidates (base content first, then the mods sorted by name).
2. Reads and validates each `mod.toml`.
3. Resolves dependencies and computes the load order.
4. Unmounts the whole VFS and mounts the base content and, on top of it, the
   active mods.
5. Persists the list of active mods and produces a report.

An invalid mod **never** aborts start-up: it is marked `Invalid` with a clear
message and the rest keep loading.

## `mod.toml`

```toml
id = "my_mod"                 # required, unique
name = "My Mod"               # required
version = "1.0.0"             # required, semver (x.y.z)
author = "Your name"          # optional
license = "CC BY-SA 4.0"      # required
description = "What it does"  # optional
min_game_version = "0.1.0"    # optional; semver or "*"
entry_script = "scripts/main.lua"  # optional
priority = 10                 # optional; higher = mounted further up
dependencies = [
    "other_mod",
    "library >=1.2.0",
    "^other_lib 1.4.0",
]
```

### Fields

| Field | Required | Notes |
|---|---|---|
| `id` | Yes | lowercase letters, digits, `-` and `_`, starting with a letter |
| `name` | Yes | Display name |
| `version` | Yes | Valid semver (`major.minor.patch[-prerelease]`) |
| `license` | Yes | SPDX or descriptive string |
| `author` | No | |
| `description` | No | |
| `min_game_version` | No | Semver or `"*"`. If it cannot be parsed, the mod is invalid |
| `entry_script` | No | Path inside the mod to an entry Lua script |
| `priority` | No | Integer; **higher priority ⇒ mounted further up** (overrides) |
| `dependencies` | No | List of `"id"` or `"id <version range>"` |

### Dependencies

They are **strings**, not tables. Each entry is an `id` optionally followed by
a space and a version range:

```toml
dependencies = ["base", "wheels >=2.0.0 <3.0.0"]
```

`resolveMods` (in `DependencyResolver`) builds a **topological, stable** load
order: a mod always comes after its dependencies. Dependencies of the
requested mods are activated automatically. The following are detected and
reported:

- missing dependencies,
- unsatisfied version ranges,
- cycles,
- duplicate ids between two mods (conflict).

Mods affected by an error are disabled; the rest load anyway.

## Version ranges

Syntax supported by `satisfies()` (all comparators are combined with AND;
alternatives are separated with `||`):

| Range | Meaning |
|---|---|
| `*` | Any version |
| `1.2.3`, `=1.2.3` | Exact match |
| `1.2`, `1` | Prefix match |
| `>=1.2.0`, `>1.2.0` | Comparison |
| `<=1.2.0`, `<1.2.0` | Comparison |
| `^1.2.3` | `>=1.2.3` and `<2.0.0` (following the `0.x` rule) |
| `~1.2.3`, `~1.2` | `>=1.2.3` and `<1.3.0` |
| `>=1.0.0 <2.0.0` | Several comparators |

Semver *build metadata* (`+something`) is ignored in comparisons.

## Priority and overriding

All mods (and the base content) are mounted at the **same virtual point**
(`/`). The mount priority determines which one wins when two mods have the
same path:

```
base (always, the lowest)
  → active mods with priority = 0 ...
  → active mods with higher priority (on top)
```

Because a higher-priority mod is mounted **later**, its files override those
of lower priority. A mod can, for example, replace a texture of the base
content without touching the engine at all.

## Enabling and disabling mods

Active mods are stored in the user configuration, under the `mods.active` key
(a list of ids). The base content is always requested, whether or not it is in
the list.

`ModManager::setModActive(id, active)` updates the list and reloads. The
configuration is saved automatically after every `reload()`.

## Report

`ModReport` (available through `ModManager::report()`) describes every
discovered mod with its state (`Active`, `Disabled`, `Invalid`), dependencies
and errors, plus a list of messages (conflicts, cycles, ...). `toString()`
returns a readable summary that the engine writes to the log.
