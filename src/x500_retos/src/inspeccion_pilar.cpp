// =============================================================================
// inspeccion_pilar (misión 2): inspecciona el pilar de un puente subiendo en
// hélice a su alrededor, con la cámara (la nariz) SIEMPRE apuntando al pilar.
//
// Mundo:   vespo up --sim --world puente_rio
// Correr:  ros2 run x500_retos inspeccion_pilar
//          ros2 run x500_retos inspeccion_pilar --ros-args -p turns:=2.0 -p speed:=1.5
//
// Viene de: orbit_mission (círculo con feedforward y yaw al centro) +
//           figure_eight (trayectoria en función del tiempo).
// Lo nuevo: el círculo además sube (una hélice) y se mide si se cumplió.
//
// El pilar está en NED norte 8.0 m, este 6.5 m del punto de despegue (los
// parámetros center_north / center_east). Mide 0.8 m de radio y tiene bandas
// de colores a 1, 2, 3 y 4 m: la hélice sube de la primera a la última.
//
//   vista desde arriba (NED)          vista de lado
//
//        x (norte)                    z (arriba)
//        ▲     .-'''-.                 4 m ─  ╭─────╮ ← sale aquí
//        │   .'   ●   '.  ← pilar             │╲   ╱│
//        │   :  centro :                      │ ╲ ╱ │   `turns` vueltas
//        │   '.       .'                      │ ╱ ╲ │
//        │     '-.▲.-'  ← inicio: el          │╱   ╲│
//        │        │       punto del círculo  1 m ─  ╰─────╯ ← entra aquí
//        ●────────┘       más cerca del despegue
//     despegue ─────► y (este)
//
// Criterio de éxito (lo mide el nodo mientras vuela la hélice):
//   - el radio real se queda en radius ± 0.3 m
//   - el yaw apunta al pilar con error < 10°
//   - de verdad dio las `turns` vueltas y llegó a z_end
//
// TU PARTE: el TODO en helix(). Mientras no lo hagas, helix() regresa el
// punto de inicio quieto: el dron llega, flota ahí lo que duraría la hélice
// y regresa. Todo lo demás (llegar, medir, regresar, aterrizar) ya funciona.
// =============================================================================

#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using x500::Setpoint;
using Point = std::array<float, 3>;

//   WARMUP -> TAKEOFF -> APPROACH -> HELIX -> EXIT -> LANDING -> DONE
enum class State { WARMUP, TAKEOFF, APPROACH, HELIX, EXIT, LANDING, DONE };

class InspeccionPilar : public rclcpp::Node
{
public:
  InspeccionPilar()
  : Node("inspeccion_pilar")
  {
    center_n_ = declare_parameter("center_north", 8.0);   // m, centro del pilar en NED
    center_e_ = declare_parameter("center_east", 6.5);
    radius_ = declare_parameter("radius", 3.0);           // m, del centro del pilar al dron
    z_start_ = declare_parameter("z_start", 1.0);         // m de altura al empezar
    z_end_ = declare_parameter("z_end", 4.0);             // m de altura al terminar
    turns_ = declare_parameter("turns", 3.0);             // vueltas (puede ser fraccionario)
    speed_ = declare_parameter("speed", 1.0);             // m/s a lo largo del círculo
    radius_tol_ = declare_parameter("radius_tolerance", 0.3);   // m
    yaw_tol_ = declare_parameter("yaw_tolerance_deg", 10.0);    // grados

    // Cuánto dura la hélice: la distancia de las vueltas entre la velocidad.
    duration_ = turns_ * 2.0 * M_PI * radius_ / speed_;
    // Ángulo de inicio (medido desde el norte hacia el este, visto desde el
    // centro del pilar): el que apunta al despegue, para llegar sin rodearlo.
    theta0_ = std::atan2(-center_e_, -center_n_);

    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "inspeccion_pilar: %.1f vueltas de radio %.1f m, de %.1f a %.1f m, %.0f s",
                turns_, radius_, z_start_, z_end_, duration_);
  }

private:
  // ---------------------------------------------------------------------------
  // La hélice en el instante t (segundos desde que empezó, 0 <= t <= duration_):
  // posición, velocidad y aceleración (feedforward) y el yaw hacia el pilar.
  // ---------------------------------------------------------------------------
  Setpoint helix(double t) const
  {
    Setpoint sp;
    // TODO: la hélice. Es el círculo de orbit_mission más una subida:
    //   - El ángulo avanza a velocidad angular w = speed_ / radius_ (rad/s),
    //     empezando en theta0_:  theta = theta0_ + w * t
    //   - En el plano: centro + radio * (cos theta, sin theta)  (x norte, y este)
    //   - La altura sube LINEAL de z_start_ a z_end_ en duration_ segundos.
    //     OJO: en NED z es hacia ABAJO, así que 4 m de altura es z = -4.
    //   - Velocidad = derivada de la posición respecto al tiempo (en x, y, z).
    //   - Aceleración (opcional pero ayuda): la del círculo apunta al centro,
    //     magnitud radius_ * w^2. En z es 0 (sube a velocidad constante).
    //   - Yaw: la nariz al pilar = el ángulo del vector dron -> centro,
    //     std::atan2(este, norte) de ese vector. sp.yawspeed (rad/s) es el
    //     feedforward de giro: ¿a qué velocidad gira la dirección al pilar?
    //
    // Extra: si todo sale bien menos el yaw, que se pasa de 10° justo al
    // arrancar, piensa qué le pasa al dron cuando la velocidad salta de 0 a
    // speed_ en un instante, y cómo arrancar suave.
    //
    // Mientras tanto: quieto en el punto de inicio, mirando al pilar.
    (void)t;
    sp.position = start_point();
    sp.yaw = static_cast<float>(x500::wrap_pi(theta0_ + M_PI));
    return sp;
  }

  // El punto del círculo más cercano al despegue, a la altura z_start_.
  Point start_point() const
  {
    return {static_cast<float>(center_n_ + radius_ * std::cos(theta0_)),
      static_cast<float>(center_e_ + radius_ * std::sin(theta0_)),
      static_cast<float>(-z_start_)};
  }

  void loop()
  {
    const Point takeoff = {0.0f, 0.0f, static_cast<float>(-z_start_)};
    const float yaw_to_pillar = static_cast<float>(x500::wrap_pi(theta0_ + M_PI));

    switch (state_) {
      case State::WARMUP:
        px4_.offboard(hover(takeoff, yaw_to_pillar));
        // Tras ~1 s de setpoints pedir OFFBOARD y armar; y reintentar cada
        // segundo por si PX4 todavía no estaba listo (o lo rechazó).
        if (++counter_ % 20 == 0) {
          px4_.request_offboard_and_arm();
        }
        if (px4_.in_offboard()) {
          RCLCPP_INFO(get_logger(), "OFFBOARD activo, subiendo a %.1f m", z_start_);
          state_ = State::TAKEOFF;
        }
        return;

      case State::TAKEOFF:
        px4_.offboard(hover(takeoff, yaw_to_pillar));
        if (px4_.distance_to(takeoff) < 0.3f) {
          RCLCPP_INFO(get_logger(), "Arriba, acercándome al pilar");
          state_ = State::APPROACH;
        }
        break;

      case State::APPROACH:
        px4_.offboard(hover(start_point(), yaw_to_pillar));
        // Empezar ya quieto: llega moviéndose HACIA el pilar y, si la hélice
        // arranca antes de que frene, el radio se cierra al principio.
        if (px4_.distance_to(start_point()) < 0.2f && yaw_error_deg() < yaw_tol_ &&
          std::hypot(px4_.local().vx, px4_.local().vy) < 0.1)
        {
          RCLCPP_INFO(get_logger(), "En el punto de inicio, empezando la hélice");
          start_ = px4_.clock();
          state_ = State::HELIX;
        }
        break;

      case State::HELIX: {
          // Tiempo desde que empezó, con el reloj de PX4 (ver px4_link.hpp).
          const double t = px4_.clock() - start_;
          if (t >= duration_) {
            RCLCPP_INFO(get_logger(), "Hélice completa, regresando sobre el despegue");
            state_ = State::EXIT;
            break;
          }
          px4_.offboard(helix(t));
          measure();
          break;
        }

      case State::EXIT: {
          // De regreso a la vertical del despegue, a la altura final: el
          // camino es la misma línea por la que llegó, lejos del pilar.
          const Point above = {0.0f, 0.0f, static_cast<float>(-z_end_)};
          px4_.offboard(hover(above, yaw_to_pillar));
          if (px4_.distance_to(above) < 0.3f) {
            px4_.land();
            RCLCPP_INFO(get_logger(), "Aterrizando");
            state_ = State::LANDING;
          }
          break;
        }

      case State::LANDING:
        if (!px4_.armed()) {
          report();
          state_ = State::DONE;
        }
        return;

      case State::DONE:
        rclcpp::shutdown();   // misión terminada: ros2 run regresa a la terminal
        return;
    }

    if (state_ != State::LANDING && !px4_.in_offboard()) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%u): misión abortada", px4_.status().nav_state);
      state_ = State::DONE;
    }
  }

  Setpoint hover(const Point & p, float yaw) const
  {
    Setpoint sp;
    sp.position = p;
    sp.yaw = yaw;
    return sp;
  }

  // Distancia real al eje del pilar (lo que una foto vería como "radio").
  double radius_now() const
  {
    return std::hypot(px4_.local().x - center_n_, px4_.local().y - center_e_);
  }

  // Cuánto se desvía la nariz de la dirección al pilar, en grados.
  double yaw_error_deg() const
  {
    const double to_pillar = std::atan2(center_e_ - px4_.local().y, center_n_ - px4_.local().x);
    return std::abs(x500::wrap_pi(px4_.local().heading - to_pillar)) * 180.0 / M_PI;
  }

  void measure()
  {
    const double r_err = std::abs(radius_now() - radius_);
    const double y_err = yaw_error_deg();
    max_radius_err_ = std::max(max_radius_err_, r_err);
    max_yaw_err_ = std::max(max_yaw_err_, y_err);
    max_alt_ = std::max(max_alt_, static_cast<double>(-px4_.local().z));
    // Vueltas dadas: sumar cuánto cambió el ángulo alrededor del pilar.
    const double angle = std::atan2(px4_.local().y - center_e_, px4_.local().x - center_n_);
    if (has_angle_) {
      swept_ += std::abs(x500::wrap_pi(angle - last_angle_));
    }
    last_angle_ = angle;
    has_angle_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "alt=%.2f m  radio=%.2f m (error %.2f)  yaw error=%.1f°",
      -px4_.local().z, radius_now(), r_err, y_err);
  }

  void report()
  {
    const bool radius_ok = max_radius_err_ <= radius_tol_;
    const bool yaw_ok = max_yaw_err_ < yaw_tol_;
    const double laps = swept_ / (2.0 * M_PI);
    // 10 % de margen: el dron va un poco atrás del setpoint y la medición se corta al terminar.
    const bool laps_ok = laps >= turns_ * 0.9;
    const bool alt_ok = max_alt_ >= z_end_ - 0.3;
    RCLCPP_INFO(get_logger(), "Aterrizado. Resultado de la inspección:");
    RCLCPP_INFO(get_logger(), "  vueltas:               %.2f de %.2f %s", laps, turns_,
                laps_ok ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), "  altura máxima:         %.2f m de %.2f m %s", max_alt_, z_end_,
                alt_ok ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), "  error máximo de radio: %.2f m (límite %.2f) %s",
                max_radius_err_, radius_tol_, radius_ok ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), "  error máximo de yaw:   %.1f° (límite %.0f°) %s",
                max_yaw_err_, yaw_tol_, yaw_ok ? "OK" : "FALLÓ");
    const bool ok = radius_ok && yaw_ok && laps_ok && alt_ok;
    RCLCPP_INFO(get_logger(), ok ? "Misión cumplida" : "Misión NO cumplida");
  }

  x500::Px4Link px4_{this};
  rclcpp::TimerBase::SharedPtr timer_;

  double center_n_, center_e_, radius_, z_start_, z_end_, turns_, speed_;
  double radius_tol_, yaw_tol_;
  double duration_, theta0_;

  State state_ = State::WARMUP;
  int counter_ = 0;
  double start_ = 0.0;   // s, reloj de PX4
  double max_radius_err_ = 0.0, max_yaw_err_ = 0.0, max_alt_ = 0.0;
  double swept_ = 0.0, last_angle_ = 0.0;   // rad recorridos alrededor del pilar
  bool has_angle_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InspeccionPilar>());
  rclcpp::shutdown();
  return 0;
}
