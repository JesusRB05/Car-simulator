Voyager (CC BY-SA 4.0)
======================

A rear-wheel-drive raycast-wheel test car: a 1200 kg chassis on four virtual
wheels with an open-differential drivetrain, traction control and speed-limited
steering.

car.toml    Car descriptor: chassis, suspension, wheels, engine, brakes, aero.
model/      Visual model (glTF placeholder; see body.gltf).

The demo currently defines this car as CarParams in
apps/sim/src/drive/RaycastCar.hpp; car.toml documents the same parameters so a
future data-driven loader can build the car from content.

No real car or brand is referenced on purpose.
