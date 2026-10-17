Car Simulator - Base Content (CC BY-SA 4.0)

This directory is the official, community-maintained base content tree. It is
mounted by the engine at the virtual root. Files in a mod that share a path
override these (later mounts win).

Layout so far:
  mod.toml          base mod manifest (format 2, id "base.core")
  apps/main.lua     base entry application, loaded by the application on startup
  tracks/<id>/      track descriptors (track.toml) and their models
  cars/<id>/        car descriptors (car.toml) and their models
  shaders/          built SPIR-V lives next to the executable, not here

The car and the circuit are loaded from the descriptors above through the
virtual file system, exactly like a mod's: the physics is built from
tracks/sunset_circuit/track.toml and cars/voyager/car.toml.

No real car brands, cars or tracks are used anywhere on purpose.