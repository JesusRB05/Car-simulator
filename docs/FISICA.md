# Física

El motor de física es propio. Vive en `src/physics` (cuerpos rígidos, elementos
de fuerza y articulaciones) y `src/vehicle` (el vehículo). Trabaja en
**unidades del SI** y en **doble precisión** para las posiciones del mundo, con
paso fijo y sin dependencias de terceros ([`DECISIONES.md`](DECISIONES.md)).

> En esta fase el motor **no resuelve colisiones ni contactos**: se centra en la
> dinámica de cuerpos rígidos, muelles y articulaciones. Las colisiones, los
> neumáticos y el modelo de vehículo completo llegan después.

## Tipos base (`physics/PhysicsTypes.hpp`)

```cpp
using Real = double;      // escalar del motor de física
using Vec3 = glm::dvec3;
using Quat = glm::dquat;
using Mat3 = glm::dmat3;
```

Se usa `double` porque las coordenadas del mundo pueden crecer mucho y el error
en `float` se acumula de forma visible dentro de una simulación.

## Matemáticas (`physics/Math.hpp`)

Cabecera pura, sin estado, con doble precisión. Incluye:

- `skew(v)` (matriz antisimétrica, `skew(v) * w == cross(v, w)`).
- Cuaterniones: `quaternionMultiply`, `quaternionInverse`,
  `quaternionNormalize`, `rotateVector`, `quaternionFromAxisAngle`.
- Mapa exponencial `quaternionFromRotationVector(v)` y su inverso
  `quaternionRotationVector(q)`. El exponencial es exacto para velocidad angular
  constante en el intervalo, que es lo que mantiene precisas las ruedas rápidas.
- `perpendicularBasis`, `safeNormalize`, `isFinite`, `Transform`.

## Cuerpos (`physics/RigidBody.hpp`)

`BodyType` es `Static`, `Kinematic` o `Dynamic`. Solo los dinámicos responden a
fuerzas; los estáticos tienen masa inversa e inercia inversa nulas.

Formas (`ShapeType`): `Box` (`halfExtents`), `Sphere` (`radius`) y `Cylinder`
(`radius`, `halfHeight` sobre el eje +Y local). `shapeInertia()` obtiene los
momentos principales de un sólido uniforme.

`RigidBody` guarda:

- Propiedades de masa: `mass`, `inverseMass`, `principalInertia`,
  `inertiaLocal` (tensor completo) e `inverseInertiaLocal`.
- Estado: `position`, `orientation`, `linearVelocity`, `angularVelocity`.
- Acumuladores `forceAccumulator` / `torqueAccumulator`.
- Amortiguación `linearDamping` / `angularDamping` (decaimiento exponencial por
  segundo).
- Cachés de mundo `rotationCache` e `inverseInertiaWorldCache` (los refresca
  `updateCache()`).

`validateInertia()` comprueba simetría, definición positiva y la desigualdad
triangular (`I1 + I2 ≥ I3`) con un solver propio de autovalores (Jacobi cíclico,
`symmetricEigenvalues`).

## Integrador (`physics/Integrator.hpp`)

Euler **semi-implícito** (simpléctico): primero velocidades, después posiciones.

- Lineal: `v += (g + F/m)·dt`.
- Angular: el término giroscópico se resuelve con la **regla del punto medio
  implícita**,

  $$
  \omega_{n+1} = \omega_n + \Delta t\, I^{-1}\!\left(\tau - \omega_{m}\times (I\,\omega_{m})\right),
  \qquad \omega_m = \tfrac{\omega_n + \omega_{n+1}}{2}
  $$

  resuelta con unas pocas iteraciones de punto fijo. La regla del punto medio es
  simpléctica y conserva **exactamente** todo invariante cuadrático del sistema
  congelado en el paso, en particular la energía cinética y el módulo del momento
  angular, de modo que una rueda a altas revoluciones ni gana energía ni pierde
  giro.
- Posiciones: `p += v·dt` y la orientación con el mapa exponencial,
  `q = exp(½ ω Δt) · q`, renormalizada.

## Elementos de fuerza (`physics/ForceElements.hpp`)

Solo suman a los acumuladores; el integrador los consume después.

- `PointSpringDamper`: muelle-amortiguador entre dos puntos (o contra un punto
  fijo del mundo si `bodyB` es `nullptr`). La fuerza es
  `f = k·(L − L₀) + c·(v_rel · n̂)`; si se da una `Curve`, esta sustituye a `k`.
- `RotationalSpringDamper`: par torsional sobre un eje local que lleva el giro
  relativo de B respecto a A hacia `restAngle`.

`Curve` es una función `y = f(x)` por interpolación lineal entre puntos de
control, con extremos fijados; vacía equivale a `y = pendiente · x`.

## Solver de restricciones (`physics/Constraint.hpp`, `physics/Joints.hpp`)

Solver de **impulsos secuenciales (PGS)** por filas, con **warm starting** y
**restricciones blandas** al estilo Box2D. Cada fila es un escalar:

$$
\dot C = J\,v = \lambda_{A}\!\cdot v_A + \alpha_A\!\cdot \omega_A
            + \lambda_{B}\!\cdot v_B + \alpha_B\!\cdot \omega_B .
$$

El impulso aplicado cambia las velocidades así:

$$
v_A \mathrel{+}= m_A^{-1}\lambda_A\,\lambda, \qquad
\omega_A \mathrel{+}= I_A^{-1}\alpha_A\,\lambda, \quad (\text{ídem } B).
$$

### Restricción blanda (soft step)

Con frecuencia `f` y amortiguación `ζ`, la masa reducida `m = 1/K` (donde
`K = J M⁻¹ Jᵀ`):

$$
\omega = 2\pi f,\quad d = 2m\zeta\omega,\quad k = m\omega^2,\quad
\text{den} = d + \Delta t\,k,
$$

$$
\gamma = \frac{1}{\Delta t\,\text{den}},\qquad
\beta = \frac{\Delta t\,k}{\text{den}},\qquad
b = \beta\,C,
$$

$$
m_{\text{eff}} = \frac{1}{K + \gamma},\qquad
\lambda = -\,m_{\text{eff}}\left(\dot C + b + \gamma\,\lambda_{\text{acum}}\right),
$$

con `λ` limitado a `[lower, upper]`. Si `f = 0` la fila es dura (`γ = β = 0`) y
solo corrige velocidad; con `f > 0` se elimina el error de posición de forma
suave, sin la energía que inyecta un Baumgarte agresivo. Frecuencia por defecto
de las articulaciones: ~60 Hz.

### Articulaciones

| Articulación | Filas | Grados de libertad |
|---|---|---|
| `DistanceConstraint` | 1 | mantiene una distancia fija |
| `SphericalConstraint` | 3 | rótula (los anclajes coinciden) |
| `FixedConstraint` | 6 | rígida (posición + orientación) |
| `RevoluteConstraint` | 7 | bisagra: 3 posición + 2 angular + motor + tope |
| `PrismaticConstraint` | 7 | corredera: 2 posición + 3 angular + motor + tope |

Las filas con motor y con topes usan límites de impulso: el motor limita el
impulso a `± par_max·Δt` (o fuerza), y los topes solo permiten el sentido que
corrige la violación. Las articulaciones que fijan orientación llaman a
`captureReference()` una vez, después de colocar los cuerpos.

## Mundo (`physics/PhysicsWorld.hpp`)

`PhysicsWorld` posee cuerpos (`std::deque<RigidBody>` para que los punteros sean
estables), restricciones y elementos de fuerza. Expone `createBody()`,
`createConstraint<T>(a, b, ...)` y `createForceElement<T>(a, b, ...)`.

`step(dt)` ejecuta, en este orden:

```
1. updateCache() y limpiar acumuladores
2. aplicar elementos de fuerza
3. integrateVelocities(dt)          // gravedad + giroscópico implícito
4. prepare(dt) de cada restricción
5. warmStart() de cada restricción
6. velocityIterations × solveVelocity()
7. integratePositions(dt)           // posiciones y orientaciones
```

`advance(elapsed)` consume tiempo real en subpasos de `fixedStep`. Por defecto
`gravity = (0, -9.80665, 0)`, `velocityIterations = 8`, `fixedStep = 1/1000`.

### Determinismo y snapshots

- Los IDs y el orden de creación (cuerpos, restricciones, elementos) son fijos,
  de modo que el orden de resolución es reproducible.
- `stateHash()` es un FNV-1a de 64 bits sobre todos los `double` del estado
  (posición, orientación, velocidades) más el tiempo.
- `saveSnapshot()` / `restoreSnapshot()` guardan estado de los cuerpos, impulsos
  acumulados de cada fila y el tiempo, para el reinicio interactivo y el test de
  determinismo.

## Vehículo (`vehicle/Vehicle.hpp`)

Modelo de **bicicleta cinemático determinista**, pensado para ser creíble y
reproducible, no un modelo de neumáticos completo todavía.

- 4 ruedas en orden **FL, FR, RL, RR**; las delanteras giran (`steered`), las
  traseras traccionan (`driven`).
- La dirección se limita en velocidad (`steerRate`) hacia el ángulo pedido.
- Fuerzas longitudinales: tracción desde `maxEngineTorque / radio`, frenado desde
  `maxBrakeTorque / radio`, resistencia aerodinámica (`dragCoefficient · v · |v|`)
  y rodadura.
- El freno de mano añade una fuerza de bloqueo y no permite invertir el sentido.
- Velocidad de guiñada cinemática: `ω = v / wheelBase · tan(steerAngle)`.
- El motor es una aproximación de rpm a partir de la velocidad.

### Convención de orientación del vehículo

El vehículo usa **forward = (sin h, 0, cos h)**, es decir **+Z a rumbo 0**.
Conviene tenerlo presente porque la convención general del motor es **-Z
adelante**; el render y la física de escena usan -Z, mientras que el vehículo
usa esta base propia. Al construir la matriz de modelo del coche hay que tener
en cuenta esta diferencia.

## Determinismo

La física es el corazón del determinismo del simulador:

- Paso fijo (por defecto `1/1000 s`), nunca `dt` variable.
- Un solo hilo ejecuta el paso: el orden de restricciones e impulsos es siempre
  el mismo.
- `-ffast-math` prohibido; contracción FMA desactivada; MSVC en `/fp:precise`.

Con esto, la misma secuencia de entradas produce exactamente la misma
trayectoria en cualquier máquina.

## Pruebas (`tests/test_physics.cpp`)

Suite analítica que cubre: caída libre y tiro parabólico (contra la solución
exacta), periodo del oscilador masa-muelle (< 0,5 %), deriva de energía acotada
60 s, momento angular de un trompo libre (< 0,1 % en 10 s), giroscopio a 3000 rpm
(sin ganancia de energía y precesión < 2 %), cadena de 20 eslabones (error
acotado 60 s), relación de masas 1:1000 estable 60 s, determinismo bit a bit
(20000 pasos + snapshot/restore) y un benchmark de la cadena de 20 eslabones.
