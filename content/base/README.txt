Car Simulator - Base Content (CC BY-SA 4.0)

This directory is the official, community-maintained base content tree. It is
mounted by the engine at the virtual root. Files in a mod that share a path
override these (later mounts win).

Layout so far:
  mod.toml          base mod manifest (id "base")
  scripts/main.lua  base entry script, loaded by the application on startup
  tracks/<id>/      track descriptors (track.toml) and their models
  cars/<id>/        car descriptors (car.toml) and their models
  shaders/          built SPIR-V lives next to the executable, not here

The driving demo currently builds its circuit and car procedurally. The
tracks/sunset_circuit and cars/voyager descriptors record the same parameters
so a future data-driven loader can reproduce them from content; nothing reads
them yet.

No real car brands, cars or tracks are used anywhere on purpose.