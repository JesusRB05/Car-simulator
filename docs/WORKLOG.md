# Work log

Running log of the external-review fixes and the mod-system redesign, one line
per point, so a session can be resumed after an interruption. Each point lands
as its own DCO-signed commit; the branch is pushed after every point.

Legend: **DONE** committed and pushed · **WIP** in progress · **TODO** not
started.

## Review fixes (first pass)

| Point | What | Commit | State |
|---|---|---|---|
| 1 | CI red: reformat with the pinned clang-format | `1961060` | DONE |
| 2 | Cross-platform determinism: full car manoeuvre in the probe | `96d1543` | DONE |
| 3 | Grip-limited dry braking | `1025d7a` | DONE |
| 4 | ABS shortens the stop (tyre curve scales with surface friction) | `691c918` | DONE |
| 5 | Cornering tolerance from the peak lateral grip | `63f957d` | DONE |
| 6 | Lua sandbox: pcall-proof budget, memory ceiling, one state per mod | `8861112` | DONE |
| 7 | Native plugins opt-in, SHA-256 fingerprint, Windows search mode | `3fe625b` | DONE |
| 8 | VFS per-file read ceiling (zip bombs) | `aed9b72` | DONE |
| 9 | Content from descriptors — **superseded by the M block** | `71033b5` | DONE |
| 10 | Pin every GitHub Action by commit SHA | `4b301ac` | DONE |
| 11 | DCO identity `174140037+JesusRB05@users.noreply.github.com` | (no commit) | DONE |
| 12 | Headless drive captures (`--camera/--time`) + README shots | `63c0b11` | DONE |
| 13 | Wheels raycast the track collision mesh (per-triangle BVH) | `e6a8a44` | DONE |

## Mod-system redesign (M block)

Replaces point 9. Design first (`docs/MODS.md`, `docs/MOD_FORMAT.md`), then
implement M1..M12 in order. Each Mx is its own signed commit, tests green,
pushed, and a line here.

| Point | What | Commit | State |
|---|---|---|---|
| Design | Rewrite `docs/MODS.md`, add `docs/MOD_FORMAT.md` | `5f3ba69` | DONE |
| M1 | Package format: `.simmod`, `mod.toml` v2, layout, `base.core` | `a851ed4` | DONE |
| M2 | Data-driven content (cars/tracks), namespaced ids, validation | `c0b60ef` | DONE |
| M3 | Additive composition: patches, skins, parts, conflict detector | `8ddeac6` | DONE |
| M4 | The installer: `.simmod`/folder detection, transactional install, rollback, CLI and drop-to-install | `6d62857` | DONE |
| M5 | Installer security: entry-name/binary validation, size caps, SHA-256 registry, fuzzing | `fd6586e` | DONE |
| M6 | Collections (modpacks) and profiles, plus working enable/disable | (this commit) | DONE |
| M4 | One-click install (`ModInstaller`, drag/drop, association, IPC) | — | TODO |
| M5 | Installer security: zip validation, SHA-256, script permissions | — | TODO |
| M6 | Dependencies, updates, modpacks, profiles | — | TODO |
| M7 | In-game catalog over HTTPS (`src/net`), `tools/mod-index/` | — | TODO |
| M8 | Multiplayer-ready content fingerprint + required-content compare | — | TODO |
| M9 | Creator tools: `sim mod <command>`, schemas, dev hot reload | — | TODO |
| M10 | In-game mod manager UI | — | TODO |
| M11 | Example mods in `mods/examples/` | — | TODO |
| M12 | End-to-end tests + README/guide | — | TODO |
