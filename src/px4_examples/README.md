# px4_examples

Ejemplos de ROS 2 con PX4 v1.17, comentados para aprender. Cada uno es un
nodo en `src/<nombre>.cpp`.

```bash
vespo up --sim                      # agente + PX4 SITL + QGroundControl
ros2 run px4_examples <nombre>      # en otra terminal
```

| Nodo | Qué hace | Cómo controla |
|---|---|---|
| `telemetry_listener` | Lee estado, posición, GPS, actitud y batería | Solo lectura |
| `imu_listener` | Lee giroscopio y acelerómetro | Solo lectura |
| `takeoff_mission` | Despega y se queda a 5 m | Offboard, posición |
| `square_mission` | Recorre un cuadrado de 5 m y aterriza | Offboard, posición |
| `orbit_mission` | Círculo alrededor de un punto mirando al centro | Offboard, posición + velocidad |
| `figure_eight` | Un 8 que sigue una curva en el tiempo | Offboard, posición + velocidad + aceleración |
| `velocity_control` | Tramos de velocidad: norte, este, giro, "adelante" | Offboard, velocidad (NED y marco del cuerpo) |
| `orbit_command` | Despega, orbita con el modo Orbit de PX4 y regresa | Comandos (Takeoff, Orbit, Return) |
| `goto_mission` | El cuadrado yendo a coordenadas GPS | Comandos (Takeoff, DO_REPOSITION, Return) |
| `return_home` | Se aleja y regresa a casa, mostrando cada fase del Return | Comandos (Takeoff, DO_REPOSITION, Return) |
| `change_speed` | Cambia la velocidad en pleno vuelo (2 -> 6 m/s) | Comandos (DO_CHANGE_SPEED) |
| `set_home` | Mueve la casa y aterriza ahí con Return | Comandos (DO_SET_HOME) |
| `battery_monitor` | Voltaje, %, corriente; avisa bajo un umbral y opcionalmente RTL / LAND | Lectura (+ comando opcional) |
| `flight_mode_monitor` | Imprime cada cambio de modo, armado, failsafe y sus banderas | Solo lectura |
| `gps_status` | Fix, satélites, error; dice si el GPS está listo para volar | Solo lectura |
| `battery_publisher` | Publica el % de batería en `/vespo/battery_percent` (`std_msgs/Float32`) | Topic propio |
| `battery_subscriber` | Lee `/vespo/battery_percent` y lo muestra como barra | Topic propio |
| `motor_test` | Gira UN motor (o todos en orden) con el dron desarmado | Comando (ACTUATOR_TEST) |

Los que tienen parámetros se cambian sin recompilar, p. ej.:

```bash
ros2 run px4_examples orbit_mission --ros-args -p radius:=8.0 -p speed:=3.0 -p laps:=3
ros2 run px4_examples figure_eight --ros-args -p size:=8.0 -p period:=25.0
ros2 run px4_examples orbit_command --ros-args -p radius:=10.0 -p laps:=1.5
ros2 run px4_examples battery_monitor --ros-args -p low_percent:=40.0 -p action:=rtl
ros2 run px4_examples motor_test --ros-args -p motor:=3 -p throttle:=0.2 -p seconds:=3.0
ros2 topic pub /vespo/battery_percent std_msgs/msg/Float32 "{data: 15.0}"   # mandar un valor a mano
```

**`motor_test` gira el motor de verdad aunque el dron esté desarmado:
siempre SIN HÉLICES.**

**Offboard o comandos.** En offboard el nodo calcula cada setpoint 10 veces
por segundo: control fino, pero si el nodo se cae PX4 entra en failsafe. Con
comandos PX4 vuela con sus propios modos y el nodo solo decide qué sigue.

Todos los que vuelan ARMAN el vehículo: pruébalos primero en simulación y en
el dron real sin hélices. Los de offboard toman como origen el punto donde
arrancó PX4 (el origen local), no donde está el dron.

Agregar un nodo: crea `src/<nombre>.cpp` y añade el nombre a `NODES` en
`CMakeLists.txt`.
