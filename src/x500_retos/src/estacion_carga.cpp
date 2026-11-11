// =============================================================================
// estacion_carga (misión 7): un dron de vigilancia patrulla la cerca de una
// base. Cuando la batería baja, deja la patrulla, va a su estación de carga,
// aterriza con PRECISIÓN corrigiendo con la cámara (ArUco 42), "carga" y
// regresa a patrullar justo donde se quedó.
//
// Mundo:   vespo up --sim --world base_carga --custom x500_cam_abajo
// Correr:  ros2 launch x500_retos estacion.launch.py       # cámara + detector + este nodo
//          ros2 launch x500_retos estacion.launch.py paquete:=x500_retos
//
// Viene de: battery_monitor (leer la batería), square_mission (patrulla) y
// detector_aruco. Lo nuevo: aterrizaje de precisión (el GPS solo te deja cerca).
//
//   WARMUP -> TAKEOFF -> PATROL ──(batería baja)──> TO_STATION -> PRECISION
//                          ▲                                        │
//                          │                                   LANDING -> CHARGING
//                       RESUME <── TAKEOFF_BACK <── REARM <──────────┘
//   (al terminar las vueltas) PATROL -> RETURN -> LANDING_HOME -> DONE
//
// La batería en PX4 SITL: armado, baja de 100 % a 50 % en 30 s (SIM_BAT_DRAIN
// = 60 s) y ahí se queda; al DESARMAR vuelve a 100 %. O sea que aterrizar en
// la estación y desarmar es "cargar". low_percent tiene que ser > 0.5.
//
// La estación según el "mapa" (station_north/east) tiene 1.5 m de error a
// propósito: la plataforma real está en norte 30, este 18. El GPS te lleva
// cerca; la cámara te pone encima.
//
// Criterio de éxito: aterriza a menos de 0.3 m del centro del ArUco cada vez
// que carga, termina las vueltas de patrulla y aterriza en casa.
//
// TU PARTE: el TODO en precision_setpoint(). Mientras no lo hagas, el dron
// baja derecho sobre la coordenada del mapa y aterriza ~1.5 m chueco.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <vector>

#include "px4_msgs/msg/battery_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "x500_retos/aruco_tools.hpp"
#include "x500_retos/msg/aruco_array.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::BatteryStatus;
using x500::kNaN;
using x500::Setpoint;
using x500_retos::msg::ArucoArray;

enum class State
{
  WARMUP, TAKEOFF, PATROL, TO_STATION, PRECISION, LANDING, CHARGING, REARM, TAKEOFF_BACK, RESUME,
  RETURN, LANDING_HOME, DONE
};

class EstacionCarga : public rclcpp::Node
{
public:
  EstacionCarga()
  : Node("estacion_carga")
  {
    altitude_ = declare_parameter("altitude", 8.0);            // m de patrulla
    speed_ = declare_parameter("speed", 5.0);                  // m/s
    low_ = declare_parameter("low_percent", 0.6);              // 0..1, tiene que ser > 0.5 en SITL
    laps_ = declare_parameter("laps", 1);
    station_n_ = declare_parameter("station_north", 30.0);     // el "mapa" (con error)
    station_e_ = declare_parameter("station_east", 19.5);
    check_n_ = declare_parameter("check_north", 30.0);         // la verdad: solo para calificar
    check_e_ = declare_parameter("check_east", 18.0);
    marker_id_ = declare_parameter("marker_id", 42);
    marker_h_ = declare_parameter("marker_height", 0.26);      // m: la plataforma
    descent_ = declare_parameter("descent_speed", 0.5);        // m/s
    charge_time_ = declare_parameter("charge_time", 5.0);      // s en la estación

    // Patrulla: un rectángulo a 4 m de la cerca, alejado del hangar y la comandancia.
    patrol_ = {{-2.0, -31.0}, {42.0, -31.0}, {42.0, 31.0}, {-2.0, 31.0}};

    battery_sub_ = create_subscription<BatteryStatus>(
      "/fmu/out/battery_status_v1", rclcpp::QoS(1).best_effort(),
      [this](const BatteryStatus::SharedPtr msg) {battery_ = msg->remaining;});
    aruco_sub_ = create_subscription<ArucoArray>(
      "/x500/aruco", 10, [this](const ArucoArray::SharedPtr msg) {on_aruco(*msg);});
    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "estacion_carga: %ld vuelta(s) de patrulla; a la estación con batería < %.0f %%",
                laps_, low_ * 100.0);
  }

private:
  // Cada imagen: si está el ArUco de la estación, dónde está en NED.
  void on_aruco(const ArucoArray & msg)
  {
    const auto & l = px4_.local();
    for (const auto & m : msg.markers) {
      if (m.id != marker_id_) {
        continue;
      }
      const double height = -l.z - marker_h_;
      const auto off = x500::marker_offset_ned(m, height, px4_.attitude().q);
      marker_n_ = l.x + off[0];
      marker_e_ = l.y + off[1];
      marker_off_ = std::hypot(off[0], off[1]);
      marker_seen_ = px4_.clock();
    }
  }

  bool marker_visible() const {return px4_.clock() - marker_seen_ < 0.5;}

  // ---------------------------------------------------------------------------
  // Aterrizaje de precisión: el setpoint de cada ciclo mientras baja.
  // ---------------------------------------------------------------------------
  Setpoint precision_setpoint() const
  {
    Setpoint sp;
    sp.yaw = 0.0f;
    // TODO: bajar SOBRE el marcador, no sobre la coordenada del mapa.
    //   Si marker_visible(): (marker_n_, marker_e_) es dónde está el ArUco en
    //   NED y marker_off_ qué tan lejos está horizontalmente del dron.
    //   - en x, y: posición = el marcador
    //   - en z: bajar (velocidad +descent_ en NED, z positivo es hacia abajo)
    //     SOLO si ya estás centrado (marker_off_ < 0.2 m); si no, quedarte a
    //     la misma altura (velocidad 0) mientras se centra
    //   - si no lo ves: quieto donde estás (y quizá subir un poco para verlo)
    //   Mezclar posición en x, y con velocidad en z se vale: deja la z de
    //   position en kNaN y las x, y de velocity en kNaN.
    //
    // Mientras tanto: derecho para abajo sobre el mapa.
    sp.position = {static_cast<float>(station_n_), static_cast<float>(station_e_), kNaN};
    sp.velocity = {kNaN, kNaN, static_cast<float>(descent_)};
    return sp;
  }

  void loop()
  {
    const auto & l = px4_.local();

    switch (state_) {
      case State::WARMUP:
      case State::REARM: {
          // Offboard desde el suelo: setpoints ~1 s y luego OFFBOARD + armar
          // (reintentando cada segundo). Sirve igual en casa y en la estación.
          px4_.offboard(hold_at(l.x, l.y, altitude_));
          if (++counter_ % 20 == 0) {
            px4_.request_offboard_and_arm();
          }
          if (px4_.in_offboard()) {
            hold_n_ = l.x;
            hold_e_ = l.y;
            RCLCPP_INFO(get_logger(), "%s, subiendo a %.0f m (batería %.0f %%)",
                        state_ == State::WARMUP ? "Despegando" : "Cargado", altitude_, battery_ * 100.0);
            state_ = state_ == State::WARMUP ? State::TAKEOFF : State::TAKEOFF_BACK;
          }
          return;
        }

      case State::TAKEOFF:
      case State::TAKEOFF_BACK:
        px4_.offboard(hold_at(hold_n_, hold_e_, altitude_));
        if (-l.z > altitude_ - 0.5) {
          if (state_ == State::TAKEOFF) {
            RCLCPP_INFO(get_logger(), "Arriba. Patrullando");
            state_ = State::PATROL;
          } else {
            RCLCPP_INFO(get_logger(), "Regresando a patrullar desde donde me quedé (norte %.1f, este %.1f)",
                        resume_n_, resume_e_);
            state_ = State::RESUME;
          }
        }
        break;

      case State::PATROL: {
          if (battery_ < low_ && battery_ > 0.0) {
            resume_n_ = l.x;
            resume_e_ = l.y;
            RCLCPP_WARN(get_logger(), "Batería en %.0f %%: dejo la patrulla en el tramo %zu y voy a cargar",
                        battery_ * 100.0, leg_ + 1);
            state_ = State::TO_STATION;
            break;
          }
          const auto & wp = patrol_[leg_ % patrol_.size()];
          px4_.offboard(go_toward(wp[0], wp[1], altitude_));
          if (std::hypot(wp[0] - l.x, wp[1] - l.y) < 1.0) {
            ++leg_;
            if (leg_ == patrol_.size() * static_cast<size_t>(laps_) + 1) {
              RCLCPP_INFO(get_logger(), "Patrulla completa (%ld vuelta/s). Regresando a casa", laps_);
              state_ = State::RETURN;
            }
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "patrulla tramo %zu, batería %.0f %%",
                               leg_ + 1, battery_ * 100.0);
          break;
        }

      case State::TO_STATION:
        px4_.offboard(go_toward(station_n_, station_e_, altitude_));
        if (std::hypot(station_n_ - l.x, station_e_ - l.y) < 0.5) {
          RCLCPP_INFO(get_logger(), "Sobre la estación (según el mapa). %s",
                      marker_visible() ? "Veo el ArUco: aterrizaje de precisión" : "No veo el ArUco todavía");
          state_ = State::PRECISION;
        }
        break;

      case State::PRECISION: {
          px4_.offboard(precision_setpoint());
          const double height = -l.z - marker_h_;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "bajando: %.1f m sobre la plataforma, "
                               "ArUco a %.2f m %s", height, marker_off_, marker_visible() ? "" : "(no lo veo)");
          if (height < 0.8) {
            // Ya casi: que PX4 termine el aterrizaje (detecta el contacto y desarma).
            px4_.land();
            state_ = State::LANDING;
          }
          break;
        }

      case State::LANDING:
        if (!px4_.armed()) {
          const double err = std::hypot(l.x - check_n_, l.y - check_e_);
          landing_errors_.push_back(err);
          RCLCPP_INFO(get_logger(), "En la estación, a %.2f m del centro %s. Cargando %.0f s...", err,
                      err < 0.3 ? "OK" : "(chueco)", charge_time_);
          charge_start_ = px4_.clock();
          state_ = State::CHARGING;
        }
        return;

      case State::CHARGING:
        // Desarmado: el reloj de PX4 sigue corriendo y la batería ya es 100 %.
        if (px4_.clock() - charge_start_ >= charge_time_ && battery_ > 0.95) {
          counter_ = 0;
          state_ = State::REARM;
        }
        return;

      case State::RESUME:
        px4_.offboard(go_toward(resume_n_, resume_e_, altitude_));
        if (std::hypot(resume_n_ - l.x, resume_e_ - l.y) < 1.0) {
          RCLCPP_INFO(get_logger(), "De vuelta en la patrulla (tramo %zu)", leg_ + 1);
          state_ = State::PATROL;
        }
        break;

      case State::RETURN:
        px4_.offboard(go_toward(0.0, 0.0, altitude_));
        if (std::hypot(l.x, l.y) < 0.5) {
          px4_.land();
          state_ = State::LANDING_HOME;
        }
        break;

      case State::LANDING_HOME:
        if (!px4_.armed()) {
          report();
          state_ = State::DONE;
        }
        return;

      case State::DONE:
        rclcpp::shutdown();
        return;
    }

    // Los estados de arriba que llegan aquí controlan en offboard; si alguien
    // le quitó el control (o ya se mandó Land), no seguimos mandando nada.
    if (state_ != State::LANDING && state_ != State::LANDING_HOME && !px4_.in_offboard()) {
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

  // Avanzar hacia (n, e) a speed_ m/s (un punto 1 s adelante + feedforward).
  Setpoint go_toward(double n, double e, double alt) const
  {
    const auto & l = px4_.local();
    const double dn = n - l.x, de = e - l.y, d = std::hypot(dn, de);
    Setpoint sp = hold_at(n, e, alt);
    if (d > speed_) {
      sp.position[0] = static_cast<float>(l.x + dn / d * speed_);
      sp.position[1] = static_cast<float>(l.y + de / d * speed_);
      sp.velocity = {static_cast<float>(dn / d * speed_), static_cast<float>(de / d * speed_), 0.0f};
    }
    return sp;
  }

  void report()
  {
    RCLCPP_INFO(get_logger(), "En casa y desarmado. Resultado:");
    bool ok = !landing_errors_.empty();
    for (size_t i = 0; i < landing_errors_.size(); ++i) {
      const bool hit = landing_errors_[i] < 0.3;
      ok = ok && hit;
      RCLCPP_INFO(get_logger(), "  carga %zu: aterrizó a %.2f m del centro (máximo 0.3) %s", i + 1,
                  landing_errors_[i], hit ? "OK" : "FALLÓ");
    }
    if (landing_errors_.empty()) {
      RCLCPP_INFO(get_logger(), "  nunca fue a cargar FALLÓ");
    }
    RCLCPP_INFO(get_logger(), ok ? "Misión cumplida" : "Misión NO cumplida");
  }

  x500::Px4Link px4_{this};
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<ArucoArray>::SharedPtr aruco_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double altitude_, speed_, low_, station_n_, station_e_, check_n_, check_e_, marker_h_, descent_;
  double charge_time_;
  int64_t laps_, marker_id_;
  std::vector<std::array<double, 2>> patrol_;

  State state_ = State::WARMUP;
  int counter_ = 0;
  size_t leg_ = 0;
  float battery_ = -1.0f;
  double hold_n_ = 0.0, hold_e_ = 0.0, resume_n_ = 0.0, resume_e_ = 0.0;
  double marker_n_ = 0.0, marker_e_ = 0.0, marker_off_ = 0.0, marker_seen_ = -1e9;
  double charge_start_ = 0.0;
  std::vector<double> landing_errors_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstacionCarga>());
  rclcpp::shutdown();
  return 0;
}
