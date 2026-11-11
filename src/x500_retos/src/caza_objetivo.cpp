// =============================================================================
// caza_objetivo (misión 6): busca a una camioneta sospechosa con la cámara de
// abajo, la identifica por el ArUco de su techo (id 7, no los señuelos), da
// su ubicación GPS y la sigue desde arriba mientras se mueve.
//
// Mundo:   vespo up --sim --world zona_bodegas --custom x500_cam_abajo
// Correr:  ros2 launch x500_retos caza.launch.py           # cámara + detector + este nodo
//          ros2 launch x500_retos caza.launch.py paquete:=x500_retos
//
// Viene de: square_mission (waypoints), figure_eight (feedforward) y el
// detector_aruco (percepción). Lo nuevo: perseguir algo que se MUEVE.
//
//   WARMUP -> TAKEOFF -> SEARCH (barrido) -> FOLLOW -> RETURN -> LANDING -> DONE
//                          ▲                  │
//                          └──── LOST ◄───────┘ (si lo pierde de vista)
//
// Barrido "podadora" a 15 m: carriles norte-sur separados 25 m (la cámara ve
// ~35 m de ancho a esa altura), sobre todo el patio. La camioneta da vueltas
// a un anillo vial a 3 m/s, así que tarde o temprano pasa por debajo.
//
// Para seguir algo que se mueve no basta con ir a donde se vio: para cuando
// llegas ya no está ahí. Hay que estimar su VELOCIDAD y mandar
//   posición = donde estará (la vista + velocidad * adelanto)
//   velocidad = la suya (feedforward, como en figure_eight)
//
// Criterio de éxito: durante follow_time (60 s) la tiene a la vista >= 80 %
// del tiempo y a menos de 1.5 m en promedio (horizontal), y regresa a casa.
//
// TU PARTE: TODO 1 (estimar la velocidad, en on_aruco) y TODO 2 (el setpoint
// de seguimiento, en follow_setpoint). Sin ellos el dron va a donde la VIO,
// se queda atrás y el promedio de error no pasa.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "x500_retos/aruco_tools.hpp"
#include "x500_retos/msg/aruco_array.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using x500::Setpoint;
using x500_retos::msg::ArucoArray;

enum class State { WARMUP, TAKEOFF, SEARCH, FOLLOW, LOST, RETURN, LANDING, DONE };

class CazaObjetivo : public rclcpp::Node
{
public:
  CazaObjetivo()
  : Node("caza_objetivo")
  {
    target_id_ = declare_parameter("target_id", 7);
    search_alt_ = declare_parameter("search_altitude", 15.0);   // m
    follow_alt_ = declare_parameter("follow_altitude", 12.0);   // m
    marker_h_ = declare_parameter("marker_height", 1.9);        // m: el ArUco va en el techo
    search_speed_ = declare_parameter("search_speed", 5.0);     // m/s
    follow_time_ = declare_parameter("follow_time", 60.0);      // s siguiéndola
    lead_ = declare_parameter("lead", 0.5);                     // s de adelanto al predecir
    lost_timeout_ = declare_parameter("lost_timeout", 3.0);     // s sin verla = perdida

    // Carriles del barrido (NED): este -37.5, -12.5, 12.5, 37.5, de norte 0 a 78.
    for (int k = 0; k < 4; ++k) {
      const double east = -37.5 + 25.0 * k;
      const bool up = k % 2 == 0;
      lanes_.push_back({up ? 0.0 : 78.0, east});
      lanes_.push_back({up ? 78.0 : 0.0, east});
    }

    sub_ = create_subscription<ArucoArray>(
      "/x500/aruco", 10, [this](const ArucoArray::SharedPtr msg) {on_aruco(*msg);});
    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "caza_objetivo: buscando el ArUco %ld", target_id_);
  }

private:
  // ---------------------------------------------------------------------------
  // Cada imagen procesada por el detector: ¿está el objetivo? ¿dónde?
  // ---------------------------------------------------------------------------
  void on_aruco(const ArucoArray & msg)
  {
    if (!px4_.local_ready()) {
      return;
    }
    const auto & l = px4_.local();
    for (const auto & m : msg.markers) {
      if (m.id != target_id_) {
        if (decoys_.insert(m.id).second) {
          RCLCPP_INFO(get_logger(), "Veo el ArUco %d: es un señuelo, lo ignoro", m.id);
        }
        continue;
      }
      // Dónde está en NED: el dron + el desplazamiento que ve la cámara.
      const double height = -l.z - marker_h_;
      const auto off = x500::marker_offset_ned(m, height, px4_.attitude().q);
      const double t = px4_.clock();
      const double n = l.x + off[0], e = l.y + off[1];
      offset_now_ = std::hypot(off[0], off[1]);
      // TODO 1: estimar la velocidad del objetivo (vel_n_, vel_e_ en m/s).
      //   Tienes la posición anterior (target_n_, target_e_) y cuándo se vio
      //   (seen_at_), y la nueva (n, e) en el instante t.
      //   - velocidad cruda = (nueva - anterior) / (t - seen_at_)
      //   - pero cada medición tiene ruido: suavízala con un filtro
      //     paso-bajas:  vel = 0.3 * cruda + 0.7 * vel
      //   - solo si la anterior es reciente (has_target_ y t - seen_at_ < 2 s)
      //     y no demasiado cercana (t - seen_at_ > 0.25 s, si no el ruido manda)
      //   Ojo: aquí abajo se actualizan target_n_/e_ y seen_at_, así que
      //   calcula ANTES de esas líneas.
      target_n_ = n;
      target_e_ = e;
      seen_at_ = t;
      if (!has_target_) {
        const auto gps = x500::gps_offset(px4_.global().lat, px4_.global().lon, off[0], off[1]);
        RCLCPP_WARN(get_logger(), "¡OBJETIVO ENCONTRADO! ArUco %d en NED norte %.1f, este %.1f "
                    "(lat %.7f, lon %.7f)", m.id, n, e, gps[0], gps[1]);
      }
      has_target_ = true;
    }
  }

  // ---------------------------------------------------------------------------
  // Hacia dónde volar para seguir al objetivo.
  // ---------------------------------------------------------------------------
  Setpoint follow_setpoint() const
  {
    Setpoint sp;
    const float z = static_cast<float>(-follow_alt_);
    sp.yaw = 0.0f;
    // TODO 2: no vayas a donde estaba, ve a donde VA a estar.
    //   - la última vista tiene una antigüedad: age = px4_.clock() - seen_at_
    //   - predicción = target + vel * (age + lead_)   (en norte y en este)
    //   - sp.position = {predicción norte, predicción este, z}
    //   - sp.velocity = {vel_n_, vel_e_, 0}  (feedforward: así no se queda atrás)
    //
    // Mientras tanto: a donde se vio por última vez.
    sp.position = {static_cast<float>(target_n_), static_cast<float>(target_e_), z};
    return sp;
  }

  void loop()
  {
    const auto & l = px4_.local();
    const double age = has_target_ ? px4_.clock() - seen_at_ : 1e9;

    switch (state_) {
      case State::WARMUP:
        px4_.offboard(hold_at(0.0, 0.0, search_alt_));
        if (++counter_ % 20 == 0) {
          px4_.request_offboard_and_arm();
        }
        if (px4_.in_offboard()) {
          RCLCPP_INFO(get_logger(), "OFFBOARD activo, subiendo a %.0f m", search_alt_);
          state_ = State::TAKEOFF;
        }
        return;

      case State::TAKEOFF:
        px4_.offboard(hold_at(0.0, 0.0, search_alt_));
        if (-l.z > search_alt_ - 0.5) {
          RCLCPP_INFO(get_logger(), "Arriba. Barriendo el patio");
          state_ = State::SEARCH;
        }
        break;

      case State::SEARCH: {
          if (age < 0.5) {
            RCLCPP_INFO(get_logger(), "Siguiéndolo durante %.0f s", follow_time_);
            follow_start_ = px4_.clock();
            state_ = State::FOLLOW;
            break;
          }
          const auto & wp = lanes_[lane_];
          if (std::hypot(wp[0] - l.x, wp[1] - l.y) < 1.5) {
            lane_ = (lane_ + 1) % lanes_.size();
          }
          px4_.offboard(go_toward(wp[0], wp[1], search_alt_, search_speed_));
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "buscando... (carril %zu)", lane_ / 2 + 1);
          break;
        }

      case State::FOLLOW: {
          const double t = px4_.clock() - follow_start_;
          if (t >= follow_time_) {
            RCLCPP_INFO(get_logger(), "Fin del seguimiento. Regresando a casa");
            state_ = State::RETURN;
            break;
          }
          if (age > lost_timeout_) {
            RCLCPP_WARN(get_logger(), "Lo perdí de vista hace %.1f s: voy a donde debería estar", age);
            lost_since_ = px4_.clock();
            state_ = State::LOST;
            break;
          }
          px4_.offboard(follow_setpoint());
          // Calificación: ¿lo veo ahora? ¿qué tan lejos está de quedar abajo?
          ++ticks_;
          if (age < 0.5) {
            ++visible_ticks_;
            error_sum_ += offset_now_;
            ++error_n_;
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "siguiendo: a %.1f m, va a %.1f m/s (%.0f s)", offset_now_, std::hypot(vel_n_, vel_e_), t);
          break;
        }

      case State::LOST: {
          if (age < 0.5) {
            RCLCPP_INFO(get_logger(), "¡Lo recuperé!");
            state_ = State::FOLLOW;
            break;
          }
          if (px4_.clock() - lost_since_ > 8.0) {
            RCLCPP_WARN(get_logger(), "No aparece: de vuelta al barrido");
            has_target_ = false;
            state_ = State::SEARCH;
            break;
          }
          ++ticks_;   // el tiempo perdido también cuenta en la calificación
          px4_.offboard(hold_at(target_n_ + vel_n_ * age, target_e_ + vel_e_ * age, search_alt_));
          break;
        }

      case State::RETURN:
        px4_.offboard(go_toward(0.0, 0.0, search_alt_, search_speed_));
        if (std::hypot(l.x, l.y) < 0.5) {
          px4_.land();
          RCLCPP_INFO(get_logger(), "Sobre casa, aterrizando");
          state_ = State::LANDING;
        }
        break;

      case State::LANDING:
        if (!px4_.armed()) {
          report();
          state_ = State::DONE;
        }
        return;

      case State::DONE:
        rclcpp::shutdown();
        return;
    }

    if (state_ != State::LANDING && !px4_.in_offboard()) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%u): misión abortada", px4_.status().nav_state);
      state_ = State::DONE;
    }
  }

  Setpoint hold_at(double n, double e, double alt) const
  {
    Setpoint sp;
    sp.position = {static_cast<float>(n), static_cast<float>(e), static_cast<float>(-alt)};
    sp.yaw = 0.0f;
    return sp;
  }

  // Avanzar hacia (n, e) a `speed` m/s: un punto que va 1 s adelante del dron
  // más la velocidad como feedforward. Así no vuela a toda velocidad entre
  // carriles (la cámara necesita tiempo para ver).
  Setpoint go_toward(double n, double e, double alt, double speed) const
  {
    const auto & l = px4_.local();
    const double dn = n - l.x, de = e - l.y, d = std::hypot(dn, de);
    Setpoint sp = hold_at(n, e, alt);
    if (d > speed) {
      sp.position[0] = static_cast<float>(l.x + dn / d * speed);
      sp.position[1] = static_cast<float>(l.y + de / d * speed);
      sp.velocity = {static_cast<float>(dn / d * speed), static_cast<float>(de / d * speed), 0.0f};
    }
    return sp;
  }

  void report()
  {
    const double visible = ticks_ > 0 ? 100.0 * visible_ticks_ / ticks_ : 0.0;
    const double error = error_n_ > 0 ? error_sum_ / error_n_ : 1e9;
    const bool found = ticks_ > 0;
    const bool ok = found && visible >= 80.0 && error < 1.5;
    RCLCPP_INFO(get_logger(), "Aterrizado. Resultado de la caza:");
    RCLCPP_INFO(get_logger(), "  objetivo encontrado:   %s", found ? "sí OK" : "no FALLÓ");
    RCLCPP_INFO(get_logger(), "  a la vista:            %.0f %% del seguimiento (mínimo 80) %s", visible,
                visible >= 80.0 ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), "  distancia promedio:    %.2f m (máximo 1.5) %s", error_n_ ? error : 0.0,
                error < 1.5 ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), ok ? "Misión cumplida" : "Misión NO cumplida");
  }

  x500::Px4Link px4_{this};
  rclcpp::Subscription<ArucoArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int64_t target_id_;
  double search_alt_, follow_alt_, marker_h_, search_speed_, follow_time_, lead_, lost_timeout_;
  std::vector<std::array<double, 2>> lanes_;
  std::set<int> decoys_;

  State state_ = State::WARMUP;
  int counter_ = 0;
  size_t lane_ = 0;
  bool has_target_ = false;
  double target_n_ = 0.0, target_e_ = 0.0, seen_at_ = 0.0;   // última vista (reloj de PX4)
  double vel_n_ = 0.0, vel_e_ = 0.0;                          // velocidad estimada, m/s
  double offset_now_ = 0.0;                                   // m del dron al objetivo
  double follow_start_ = 0.0, lost_since_ = 0.0;
  int ticks_ = 0, visible_ticks_ = 0, error_n_ = 0;
  double error_sum_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CazaObjetivo>());
  rclcpp::shutdown();
  return 0;
}
