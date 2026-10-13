Sunset Circuit (CC BY-SA 4.0)
=============================

A short closed circuit used by the driving demo, with a long main straight, a
hairpin, a fast sweeper and a chicane, laid over one smooth hill.

track.toml  Track descriptor: control points, surface widths and terrain.
model/      Visual model (glTF placeholder; see track.gltf).

The demo currently builds this circuit procedurally in
apps/sim/src/drive/Track.cpp; track.toml documents the same parameters so a
future data-driven loader can load it from content. Static collision scenery
(barrier walls and a run-off test yard) is generated procedurally too, so cars
can be crash-tested without leaving the asphalt.

No real track or brand is referenced on purpose.
