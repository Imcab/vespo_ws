// =============================================================================
// flight_mode_monitor: imprime CADA CAMBIO de modo de vuelo, de armado y de
// failsafe. No imprime nada mientras nada cambia.
//
// Sirve para entender qué hace PX4 mientras corre otro nodo o mientras vuelas
// con QGroundControl: "pasó de Takeoff a Hold", "entró en failsafe por
// pérdida de offboard", etc.
//
// Lee:
//   /fmu/out/vehicle_status_v1  nav_state (modo activo), nav_state_user_intention
//                               (el que pidió el usuario), arming_state, failsafe
//   /fmu/out/failsafe_flags     una bandera bool por cada problema que PX4 vigila
//
// Ejecutar (no arma ni manda nada, se puede dejar corriendo siempre):
//   ros2 run px4_examples flight_mode_monitor
// =============================================================================

#include <string>
#include <utility>   // std::pair
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/failsafe_flags.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::FailsafeFlags;

class FlightModeMonitor : public rclcpp::Node
{
public:
  FlightModeMonitor()
  : Node("flight_mode_monitor")
  {
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {on_status(*msg);});
    // FailsafeFlags tiene MESSAGE_VERSION = 0: sin sufijo _vN.
    failsafe_sub_ = create_subscription<FailsafeFlags>(
      "/fmu/out/failsafe_flags", qos,
      [this](const FailsafeFlags::SharedPtr msg) {on_failsafe_flags(*msg);});

    RCLCPP_INFO(get_logger(), "flight_mode_monitor: esperando a PX4...");
  }

private:
  void on_status(const VehicleStatus & s)
  {
    // first_ = true solo con el primer mensaje: imprime el estado inicial.
    if (first_ || s.arming_state != last_.arming_state) {
      RCLCPP_INFO(get_logger(), "%s",
                  s.arming_state == VehicleStatus::ARMING_STATE_ARMED ? "ARMADO" : "DESARMADO");
    }
    if (first_ || s.nav_state != last_.nav_state) {
      if (first_) {
        RCLCPP_INFO(get_logger(), "Modo: %s", mode_name(s.nav_state));
      } else {
        RCLCPP_INFO(get_logger(), "Modo: %s -> %s", mode_name(last_.nav_state),
                    mode_name(s.nav_state));
      }
      // Si el modo activo no es el que pidió el usuario, PX4 lo cambió solo.
      if (s.nav_state != s.nav_state_user_intention) {
        RCLCPP_WARN(get_logger(), "  (el usuario pidió %s, PX4 está en %s)",
                    mode_name(s.nav_state_user_intention), mode_name(s.nav_state));
      }
    }
    if (!first_ && s.failsafe != last_.failsafe) {
      if (s.failsafe) {
        RCLCPP_ERROR(get_logger(), "FAILSAFE ACTIVO (modo %s)", mode_name(s.nav_state));
      } else {
        RCLCPP_INFO(get_logger(), "Failsafe terminado");
      }
    }
    if (first_ || s.pre_flight_checks_pass != last_.pre_flight_checks_pass) {
      RCLCPP_INFO(get_logger(), "Chequeos de pre-vuelo: %s",
                  s.pre_flight_checks_pass ? "OK, se puede armar" : "FALLAN, no se puede armar");
    }
    last_ = s;
    first_ = false;
  }

  void on_failsafe_flags(const FailsafeFlags & f)
  {
    // Lista de (nombre, valor) de las banderas más útiles. Hay más en
    // px4_msgs/msg/FailsafeFlags.msg.
    const std::vector<std::pair<const char *, bool>> flags = {
      {"sin posición local", f.local_position_invalid},
      {"sin posición global (GPS)", f.global_position_invalid},
      {"sin altitud", f.local_altitude_invalid},
      {"sin home", f.home_position_invalid},
      {"señal offboard perdida", f.offboard_control_signal_lost},
      {"control RC perdido", f.manual_control_signal_lost},
      {"conexión con QGC perdida", f.gcs_connection_lost},
      {"batería: poco tiempo de vuelo", f.battery_low_remaining_time},
      {"batería en mal estado", f.battery_unhealthy},
      {"geocerca violada", f.geofence_breached},
      {"viento excesivo", f.wind_limit_exceeded},
      {"tiempo de vuelo máximo", f.flight_time_limit_exceeded},
      {"falla crítica (actitud)", f.fd_critical_failure},
      {"falla de motor", f.fd_motor_failure},
    };

    // Junta en un texto las que están activas: "sin home, control RC perdido".
    std::string active;
    for (const auto & [name, value] : flags) {   // "structured binding" de C++17
      if (value) {
        active += active.empty() ? name : std::string(", ") + name;
      }
    }
    if (active != last_flags_) {
      if (active.empty()) {
        RCLCPP_INFO(get_logger(), "Banderas de failsafe: ninguna");
      } else {
        RCLCPP_WARN(get_logger(), "Banderas de failsafe: %s", active.c_str());
      }
      last_flags_ = active;
    }
  }

  static const char * mode_name(uint8_t nav_state)
  {
    switch (nav_state) {
      case VehicleStatus::NAVIGATION_STATE_MANUAL: return "Manual";
      case VehicleStatus::NAVIGATION_STATE_ALTCTL: return "Altitude";
      case VehicleStatus::NAVIGATION_STATE_POSCTL: return "Position";
      case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "Mission";
      case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: return "Hold";
      case VehicleStatus::NAVIGATION_STATE_AUTO_RTL: return "Return";
      case VehicleStatus::NAVIGATION_STATE_POSITION_SLOW: return "Position Slow";
      case VehicleStatus::NAVIGATION_STATE_ALTITUDE_CRUISE: return "Altitude Cruise";
      case VehicleStatus::NAVIGATION_STATE_ACRO: return "Acro";
      case VehicleStatus::NAVIGATION_STATE_DESCEND: return "Descend";
      case VehicleStatus::NAVIGATION_STATE_TERMINATION: return "Termination";
      case VehicleStatus::NAVIGATION_STATE_OFFBOARD: return "Offboard";
      case VehicleStatus::NAVIGATION_STATE_STAB: return "Stabilized";
      case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "Takeoff";
      case VehicleStatus::NAVIGATION_STATE_AUTO_LAND: return "Land";
      case VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET: return "Follow Me";
      case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND: return "Precision Land";
      case VehicleStatus::NAVIGATION_STATE_ORBIT: return "Orbit";
      case VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: return "VTOL Takeoff";
      default:
        // EXTERNAL1..8 son modos creados desde ROS 2 (px4-ros2-interface-lib).
        if (nav_state >= VehicleStatus::NAVIGATION_STATE_EXTERNAL1 &&
          nav_state <= VehicleStatus::NAVIGATION_STATE_EXTERNAL8)
        {
          return "Externo (ROS 2)";
        }
        return "Desconocido";
    }
  }

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<FailsafeFlags>::SharedPtr failsafe_sub_;

  VehicleStatus last_;
  bool first_ = true;
  std::string last_flags_ = "?";   // "?" para que imprima la primera vez
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightModeMonitor>());
  rclcpp::shutdown();
  return 0;
}
