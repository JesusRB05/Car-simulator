# Mod system

Goal: **the easiest simulator to install and to create mods for.** One file,
one click, a manager inside the game, and content that is *added* rather than
overwritten.

We take the best of the field and avoid its mistakes:

- **BeamNG.drive** — one archive, one click, an in-game manager, and *parts*
  that plug into a vehicle without replacing its files.
- **Assetto Corsa + Content Manager** — folders per content type, skins that
  live inside the car, drag-any-file installation, and downloading a server's
  content when you join.
- **rFactor 2** — championship packs and per-component versions.

Mistakes we refuse to copy: files that clobber each other, no versions or
dependencies, encrypted data, mandatory external tools, and several
installation paths that disagree.

The formats are in [MOD_FORMAT.md](MOD_FORMAT.md); a step-by-step creator
tutorial is in [MODDING_GUIDE.md](MODDING_GUIDE.md).

## 1. Concepts

- **Package** — a mod is one `.simmod` file (a ZIP) or the same tree as a
  folder. The extension is a single constant in the code so it can be renamed.
- **Manifest** — `mod.toml` v2: namespaced `id` (`author.name`), name, semver
  `version`, `authors`, SPDX `license`, `description`, `cover`,
  `min_game_version`, `types`, `dependencies`, `conflicts`, `permissions`,
  `urls` and `format = 2`. v1 manifests are migrated in memory with a warning.
- **Content reference** — `<mod-id>/<content-id>`, always. The official content
  is the mod **`base.core`** and is described by the very same format.
- **Additive composition** — skins, parts, setups, tyres, weather, locales, apps
  and modes are merged by id; nothing is copied and nothing is replaced unless
  the mod explicitly declares an override.
- **Patches** — small, targeted changes (`set`/`add`/`multiply`/`remove`) with a
  mandatory `target`; never a silent whole-file replacement.
- **Profiles** — named sets of active mods and their load order, to switch a
  league setup in one click and to share it as a small file.

## 2. Layout

The package layout and every descriptor are specified in
[MOD_FORMAT.md](MOD_FORMAT.md): `mod.toml`, `cars/`, `tracks/` (`layouts/`),
`skins/<car>/`, `parts/`, `tyres/`, `sounds/`, `setups/`, `weather/`, `apps/`,
`modes/`, `visuals/`, `locale/`, `input/`, `plugins/`, `patches/`.

The old `content/base` becomes the mod `base.core` with exactly this layout.

## 3. What a mod can do without copying anything

- **Skins** — `skins/<car-mod>/<car>/<skin>/` shows up in the skin selector of a
  car made by *any* mod.
- **Parts and slots** — a car declares `slots` (wheels, tyre, wing, engine,
  exhaust, suspension, gear ratios, ...) and a default per slot; a mod adds a
  part to a slot of another mod's car by dropping a `part.toml`. Setups choose
  one part per slot.
- **Patches** — targeted edits of another mod's values, applied in load order.
- **Overrides** — a whole-file replacement only when declared under
  `overrides` in `mod.toml`; the manager shows it as a warning.

### Conflict detector

Two mods that patch the same `(target, path)` or override the same file are
reported with the **effective order** (who wins today). The user reorders mods
by drag-and-drop; the order is the tie-break and is saved in the active
profile.

## 4. One-click installation

There is exactly **one** code path — `ModInstaller` — and several ways to reach
it:

1. **Drag a file onto the window** (SDL file drop).
2. **Double-click a `.simmod`** (optional per-user file association, no admin
   rights). If the game is already running, the second process forwards the
   file to the first over a local IPC channel and exits.
3. **A button in the in-game browser** (section 6).
4. **A `simmod://install?id=…&version=…` link** (optional URL-scheme
   registration; always asks for confirmation).
5. **Copy into the user mods folder** — detected at start-up and while running.
6. **`sim --install-mod <path>`** (add `--dry-run` to print the plan and stop):
   what a file association, a `simmod://` link or an IPC hand-off ends up
   running, and usable on its own.

### Smart detection

If the dropped file is a ZIP **without** `mod.toml`, the installer analyses it:
when it recognises the structure (a folder with a `car.toml`, or a loose
skin), it offers to create the manifest and wrap it into a proper package;
otherwise it explains exactly what is missing.

### Where mods live

Mods are installed in the **user data directory** — never in the game folder,
which may be read-only:

- Windows: `%APPDATA%/Car Simulator/mods/`
- Linux: `$XDG_DATA_HOME/Car Simulator/mods/` (or `~/.local/share/…`)
- Configurable with `mods.directory` in `config.toml`.

The application also scans the bundled `mods/` folder next to the executable
(the official example mod lives there), below the user's own directory.

Each install keeps the previous version in a hidden backup folder, so the user
can roll back:

### Confirmation screen

Before installing: cover, name, author, version, license, size, what it
contains ("2 cars, 14 skins"), the dependencies that will be installed with it,
the script permissions it requests, and — if it ships native plugins — a strong
warning. **Native plugins are never enabled automatically.**

Uninstall and disable live in the manager; uninstall removes every file and
registry entry it created.

## 5. Installer security

Validated **before** anything is extracted or mounted:

- no `..`, no absolute paths, no drive letters or UNC in entry names;
- total size, per-file size, entry count and compression-ratio ceilings (ZIP
  bomb);
- per-folder allowed extensions (executables only inside `plugins/`);
- names that are valid on Windows **and** Linux (no reserved names, no `<>:"|?*`).

Every installed package's SHA-256 is stored in the local registry.

### Script permissions

Scripts declare what they need in `mod.toml` (`permissions = [...]`), for
example `hud`, `telemetry.read`, `input.read`, `vehicle.tuning`,
`session.control`. The Lua API exposes **only** what is declared. A script has
**no** network, **no** filesystem outside the read-only VFS of its own mod, and
**no** access to other mods except its declared dependencies. A request for an
undeclared capability is denied and logged.

The `mod.toml` parser and the ZIP validator are fuzzed (wired into the CI
security job).

## 6. Catalog (in-game mod browser)

Source: an **open static index** (a JSON/TOML file served over HTTPS, for
example a GitHub `mod-index` repository) with one entry per version: id,
version, download URL, SHA-256, size, cover, types, dependencies, license.
The files themselves can be hosted anywhere (GitHub Releases, the author's
site). Several sources can be configured.

Privacy: the catalog is **off by default**; turning it on explains what is
downloaded. There are **no accounts, no telemetry, no cookies and no
identifiers** — only GET requests for static files. Every download is verified
against its SHA-256 before installation.

The HTTP client is isolated in `src/net` and justified in
[DECISIONS.md](DECISIONS.md). `tools/mod-index/` generates the index from a
folder of packages. An index is a static file, so a local copy, a share or a
USB stick work with the same code path; the ships-today transport reads
`file://` URLs, and `https://` reports the missing transport instead of
downgrading to the clear (docs/DECISIONS.md).

The browser has **Installed / Explore / Updates** tabs, search, type filters,
covers, install/update/uninstall, download progress and clear error states.

## 7. Dependencies, updates and profiles

- semver ranges are resolved when mods are enabled; missing dependencies are
  offered for installation (from the catalog when configured).
- **Collections** (modpacks/championships) are a mod that only lists other mods
  (with exact versions or ranges) and can carry a session configuration.
  Installing one installs all of them.
- Update checks are manual by default (automatic optional), show the changelog
  and offer a rollback to the previous version.
- **Profiles** switch a named active set + order in one click, and export/import
  as a small file.

## 8. Multiplayer-ready content

No game networking is implemented yet. What exists is the **format** and the
comparison so a future session can reject mismatches early:

- a deterministic **content fingerprint** per active mod that affects the
  simulation (cars, circuits, physics-affecting patches);
- a `RequiredContent` message (`id`, `version`, `hash`, optional `url`);
- a comparator that reports `missing`, `version-mismatch` and `hash-mismatch`
  with one-click download+install.

**Skins do not require a match.**

## 9. Creator tools

A single CLI lives in `sim.exe`:

```bat
sim mod new car ana.karts/kart                 # scaffold, commented
sim mod new car ana.karts/kart --from base.core/voyager
sim mod validate mods/examples/kart            # CI-friendly, exit code
sim mod pack mods/examples/kart                # .simmod + sha256 + index entry
sim mod list | install | remove | enable | disable
sim mod skin-template base.core/voyager        # UV template PNG with guides
```

- **Developer mode** — an unpacked mods folder with hot reload when
  `car.toml`, `track.toml`, a skin or a script is saved (no session restart
  when it is safe).
- **JSON schemas** for `mod.toml`, `car.toml`, `track.toml`, `livery.toml`,
  `part.toml`, `setup.toml`, `patches` and the index, in `docs/schemas/`, so
  Taplo and *Even Better TOML* autocomplete and validate in the editor.

## 10. In-game manager

- Installed list: cover, type, version, author, origin, state (active, disabled,
  error, update available), conflicts and a load-order control.
- **Human errors**, technical detail collapsible: *"The skin 'Rojo' needs the
  car 'pepe.f3', which is not installed. [Install] [Disable]"*.
- Car and track selection shows a label with the source mod.

## 11. Example mods

`mods/examples/` is built with `mod pack` during the build and validated in CI:
a data-only front-wheel-drive car with 2 skins; a skin pack for the Voyager by
another author; a part for the Voyager added by slot; a setup patch (stiffer
springs); a Lua HUD app with declared permissions; a small circuit with two
layouts; and an example championship collection that lists them.

The old `mods/example_mod` is migrated into this set.

## 12. How a mod is born (short version)

1. `sim mod new car ana.karts/kart` (or `--from base.core/voyager`).
2. Edit `cars/kart/car.toml`; add `skins/<car>/<skin>/livery.toml`.
3. `sim mod validate .` until it is clean.
4. `sim mod pack .` → `ana.karts-1.0.0.simmod`.
5. Publish the file anywhere; open a pull request to a `mod-index` with its
   entry. Players install it by dropping it on the window or from the browser.

See [MODDING_GUIDE.md](MODDING_GUIDE.md) for the walk-through.
