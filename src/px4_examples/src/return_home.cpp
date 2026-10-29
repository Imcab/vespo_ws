// =============================================================================
// return_home: despega, se aleja de casa y le pide a PX4 regresar (Return /
// RTL). Mientras regresa muestra en qué fase va: subir, volver, bajar.
//
// Return (RTL) en un multicóptero:
//   1. Sube a la altura de retorno si está más bajo: RTL_RETURN_ALT, pero
//      recortada por un "cono" sobre casa (RTL_CONE_ANG = 45°): cerca de casa
//      sube menos (con 45°, como mucho tantos metros como esté lejos).
//   2. Vuela en línea recta a casa (home).
//   3. Baja a RTL_DESCEND_ALT, espera RTL_LAND_DELAY y aterriza.
//   4. Desarma solo al tocar tierra.
// Los valores dependen del airframe: en el x500 de SITL RTL_RETURN_ALT = 30 m
// y RTL_DESCEND_ALT = 10 m. Revisa los de tu dron en QGroundControl
// (Parámetros > Return Mode); en la cancha conviene una altura segura.
//
// "Casa" es /fmu/out/home_position_v1: PX4 la fija donde armó por primera vez.
// Para cambiarla ver set_home.cpp.
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples return_home
//   ros2 run px4_examples return_home --ros-args -p north:=40.0 -p east:=-20.0
// =============================================================================

#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/home_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::HomePosition;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;

using namespace std::chrono_literals;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kEarthRadius = 6371000.0;   // m

//   WAIT_GPS -> ARMING -> TAKEOFF -> GO_AWAY -> RETURN -> DONE
enum class State { WAIT_GPS, ARMING, TAKEOFF, GO_AWAY, RETURN, DONE };

class ReturnHome : public rclcpp::Node
{
public:
  ReturnHome()
  : Node("return_home")
  {
    altitude_ = declare_parameter("altitude", 5.0);   // m de despegue
    north_ = declare_parameter("north", 30.0);        // m hacia donde se aleja
    east_ = declare_parameter("east", 20.0);

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
    // HomePosition tiene MESSAGE_VERSION = 1 -> _v1. Trae casa en GPS
    // (lat, lon, alt) y en local (x, y, z), lo que nos ahorra convertir.
    home_sub_ = create_subscription<HomePosition>(
      "/fmu/out/home_position_v1", qos,
      [this](const HomePosition::SharedPtr msg) {home_ = *msg;});
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      [this](const VehicleCommandAck::SharedPtr msg) {
        const bool repeated = msg->command == last_ack_.command && msg->result == last_ack_.result;
        last_ack_ = *msg;
        if (!repeated && msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
          RCLCPP_WARN(get_logger(), "PX4 rechazó el comando %u (result=%u)",
                      msg->command, msg->result);
        }
      });

    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    timer_ = create_wall_timer(100ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "return_home: esperando posición GPS...");
  }

private:
  void loop()
  {
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;

    switch (state_) {
      case State::WAIT_GPS:
        if (global_.lat_lon_valid && global_.alt_valid && local_.xy_valid) {
          ground_z_ = local_.z;
          RCLCPP_INFO(get_logger(), "GPS listo, armando");
          state_ = State::ARMING;
        }
        break;

      case State::ARMING:
        if (!armed) {
          if (every_second()) {
            // Hold primero: en Land (como queda tras aterrizar) no deja armar.
            if (status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
              send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 3.0f);
            } else {
              send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
            }
          }
          break;
        }
        send_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, kNaN, kNaN, kNaN, kNaN,
                     global_.lat, global_.lon, global_.alt + static_cast<float>(altitude_));
        RCLCPP_INFO(get_logger(), "Armado, despegando a %.1f m", altitude_);
        state_ = State::TAKEOFF;
        break;

      case State::TAKEOFF:
        if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER &&
          std::abs(ground_z_ - local_.z - altitude_) < 0.5)
        {
          if (home_.valid_hpos) {
            RCLCPP_INFO(get_logger(), "Casa: %.7f, %.7f (%.1f m AMSL)", home_.lat, home_.lon,
                        home_.alt);
          }
          go_away();
          state_ = State::GO_AWAY;
        }
        break;

      case State::GO_AWAY:
        // Destino en local = casa local + (north, east).
        if (std::hypot(home_.x + north_ - local_.x, home_.y + east_ - local_.y) < 1.0) {
          RCLCPP_INFO(get_logger(), "Lejos de casa (%.0f m). Return!", distance_home());
          send_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
          state_ = State::RETURN;
        }
        break;

      case State::RETURN:
        if (!armed) {
          RCLCPP_INFO(get_logger(), "En casa y desarmado (a %.1f m del home)", distance_home());
          state_ = State::DONE;
          break;
        }
        report_return();
        break;

      case State::DONE:
        break;
    }
  }

  void go_away()
  {
    const double lat = global_.lat + (north_ / kEarthRadius) * 180.0 / M_PI;
    const double lon = global_.lon +
      (east_ / (kEarthRadius * std::cos(global_.lat * M_PI / 180.0))) * 180.0 / M_PI;
    // DO_REPOSITION como en goto_mission: -1 = velocidad por defecto de PX4.
    send_command(VehicleCommand::VEHICLE_CMD_DO_REPOSITION, -1.0f, 1.0f, 0.0f, kNaN, lat, lon,
                 global_.alt);
    RCLCPP_INFO(get_logger(), "Alejándome %.0f m N, %.0f m E", north_, east_);
  }

  // Una línea por segundo con la fase de Return en la que vamos.
  void report_return()
  {
    const double dist = distance_home();
    const double alt = ground_z_ - local_.z;
    const char * phase;
    if (status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_RTL) {
      phase = "ya no está en Return (¿alguien cambió el modo?)";
    } else if (dist > 2.0 && local_.vz < -0.3) {
      phase = "subiendo a la altura de retorno";
    } else if (dist > 2.0) {
      phase = "volando a casa";
    } else {
      phase = "sobre casa, bajando";
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s | a %.1f m de casa, altura %.1f m",
                         phase, dist, alt);
  }

  double distance_home()
  {
    return std::hypot(home_.x - local_.x, home_.y - local_.y);
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
  rclcpp::Subscription<HomePosition>::SharedPtr home_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition local_;
  VehicleGlobalPosition global_;
  HomePosition home_;
  VehicleCommandAck last_ack_;

  double altitude_, north_, east_;

  State state_ = State::WAIT_GPS;
  float ground_z_ = 0.0f;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReturnHome>());
  rclcpp::shutdown();
  return 0;
}
