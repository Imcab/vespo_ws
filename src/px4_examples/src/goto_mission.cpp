// =============================================================================
// goto_mission: el square_mission pero SIN offboard. Despega con el modo
// Takeoff de PX4 y va a cada waypoint con DO_REPOSITION (el "Go to location"
// de QGroundControl). Al final regresa a casa con Return (RTL).
//
// Lo nuevo:
//   - Pasar un waypoint en metros (norte, este) a coordenadas GPS (lat, lon),
//     que es lo que piden los comandos de PX4.
//   - DO_REPOSITION: PX4 vuela solo hasta el punto (con su propia velocidad
//     y frenado) y se queda en Hold ahí. Nosotros solo esperamos a que llegue.
//   - Igual que orbit_command: si el nodo se muere el dron se queda en Hold,
//     no hay failsafe de offboard.
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples goto_mission
//   ros2 run px4_examples goto_mission --ros-args -p altitude:=8.0 -p speed:=3.0
// =============================================================================

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;

using namespace std::chrono_literals;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kEarthRadius = 6371000.0;   // m

//   WAIT_GPS -> ARMING -> TAKEOFF -> GOTO -> RETURN -> DONE
enum class State { WAIT_GPS, ARMING, TAKEOFF, GOTO, RETURN, DONE };

class GotoMission : public rclcpp::Node
{
public:
  GotoMission()
  : Node("goto_mission")
  {
    altitude_ = declare_parameter("altitude", 5.0);   // m sobre el despegue
    speed_ = declare_parameter("speed", 2.0);         // m/s; -1 = la velocidad por defecto de PX4

    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    local_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});
    global_sub_ = create_subscription<VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos,
      [this](const VehicleGlobalPosition::SharedPtr msg) {global_ = *msg;});
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      [this](const VehicleCommandAck::SharedPtr msg) {
        // Solo avisa de rechazos, y una vez por cada respuesta distinta.
        const bool repeated = msg->command == last_ack_.command && msg->result == last_ack_.result;
        last_ack_ = *msg;
        if (!repeated && msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
          RCLCPP_WARN(get_logger(), "PX4 rechazó el comando %u (result=%u)",
                      msg->command, msg->result);
        }
      });

    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    timer_ = create_wall_timer(100ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "goto_mission: esperando posición GPS...");
  }

private:
  // Waypoints en metros {norte, este} desde el punto de despegue. La altura
  // es el parámetro `altitude`. Mismo cuadrado que square_mission.
  std::vector<std::array<double, 2>> waypoints_ = {
    {5.0, 0.0},
    {5.0, 5.0},
    {0.0, 5.0},
    {0.0, 0.0},
  };

  void loop()
  {
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;

    switch (state_) {
      case State::WAIT_GPS:
        if (global_.lat_lon_valid && global_.alt_valid && local_.xy_valid) {
          // Guardamos el punto de despegue en las dos coordenadas: GPS para
          // los comandos y local (NED) para medir si ya llegamos.
          home_lat_ = global_.lat;
          home_lon_ = global_.lon;
          home_alt_ = global_.alt;
          home_x_ = local_.x;
          home_y_ = local_.y;
          home_z_ = local_.z;
          RCLCPP_INFO(get_logger(), "GPS listo, armando");
          state_ = State::ARMING;
        }
        break;

      case State::ARMING:
        if (!armed) {
          if (every_second()) {
          // Primero a Hold (AUTO_LOITER): en algunos modos, como Land (el modo
          // en que queda tras aterrizar), PX4 no deja armar.
          // DO_SET_MODE: param1 = 1 (modo custom), param2 = 4 (AUTO), param3 = 3 (LOITER).
            if (status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
              send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 3.0f);
            } else {
              send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
            }
          }
          break;
        }
        send_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, kNaN, kNaN, kNaN, kNaN,
                     home_lat_, home_lon_, target_alt_amsl());
        RCLCPP_INFO(get_logger(), "Armado, despegando a %.1f m", altitude_);
        state_ = State::TAKEOFF;
        break;

      case State::TAKEOFF:
        if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER &&
          std::abs(home_z_ - local_.z - altitude_) < 0.5)
        {
          goto_waypoint(0);
          state_ = State::GOTO;
        }
        break;

      case State::GOTO: {
          const auto & wp = waypoints_[wp_index_];
          const double dist = std::hypot(home_x_ + wp[0] - local_.x, home_y_ + wp[1] - local_.y);
          // 0.5 m: PX4 frena solo al llegar, no hace falta ser más estrictos.
          if (dist < 0.5) {
            RCLCPP_INFO(get_logger(), "Llegué al WP %zu", wp_index_);
            if (wp_index_ + 1 == waypoints_.size()) {
              send_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
              RCLCPP_INFO(get_logger(), "Misión completa, regresando a casa");
              state_ = State::RETURN;
              break;
            }
            goto_waypoint(wp_index_ + 1);
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "WP %zu a %.1f m",
                               wp_index_, dist);
          break;
        }

      case State::RETURN:
        if (!armed) {
          RCLCPP_INFO(get_logger(), "En casa y desarmado");
          state_ = State::DONE;
        }
        break;

      case State::DONE:
        break;
    }
  }

  void goto_waypoint(size_t index)
  {
    wp_index_ = index;
    const auto & wp = waypoints_[index];

    // Metros -> grados (aproximación de Tierra plana, buena para unos km):
    //   latitud:  d_norte / R            radianes
    //   longitud: d_este / (R cos(lat))  radianes (los meridianos se juntan
    //             hacia los polos, por eso el cos de la latitud)
    const double lat = home_lat_ + (wp[0] / kEarthRadius) * 180.0 / M_PI;
    const double lon = home_lon_ +
      (wp[1] / (kEarthRadius * std::cos(home_lat_ * M_PI / 180.0))) * 180.0 / M_PI;

    // DO_REPOSITION: param1 = velocidad, param2 = 1 (obligatorio en PX4:
    // "cambia a modo Hold"), param4 = yaw en grados (NaN = no girar),
    // param5/6 = lat/lon, param7 = altitud AMSL.
    send_command(VehicleCommand::VEHICLE_CMD_DO_REPOSITION,
                 static_cast<float>(speed_), 1.0f, 0.0f, kNaN, lat, lon, target_alt_amsl());
    RCLCPP_INFO(get_logger(), "Yendo al WP %zu: %.1f m N, %.1f m E", index, wp[0], wp[1]);
  }

  float target_alt_amsl()
  {
    return home_alt_ + static_cast<float>(altitude_);
  }

  bool every_second()
  {
    if ((now() - last_command_).seconds() < 1.0) {
      return false;
    }
    last_command_ = now();
    return true;
  }

  uint64_t now_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  // Con los 7 parámetros (ver orbit_command.cpp: param5/6 son double).
  void send_command(uint32_t command, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f,
    float p4 = 0.0f, double p5 = 0.0, double p6 = 0.0, float p7 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.param3 = p3;
    msg.param4 = p4;
    msg.param5 = p5;
    msg.param6 = p6;
    msg.param7 = p7;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();
    command_pub_->publish(msg);
  }

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition local_;
  VehicleGlobalPosition global_;
  VehicleCommandAck last_ack_;

  double altitude_, speed_;

  State state_ = State::WAIT_GPS;
  size_t wp_index_ = 0;
  double home_lat_ = 0.0, home_lon_ = 0.0;
  float home_alt_ = 0.0f, home_x_ = 0.0f, home_y_ = 0.0f, home_z_ = 0.0f;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GotoMission>());
  rclcpp::shutdown();
  return 0;
}
