// =============================================================================
// set_home: cambia la posición de "casa" (home) con DO_SET_HOME y comprueba
// que Return aterriza en la casa NUEVA.
//
// Casa es a donde va el dron en Return (RTL) y en los failsafes. PX4 la fija
// sola donde arma; cambiarla sirve p. ej. para que, si algo falla, el dron
// regrese a la plataforma de aterrizaje y no al punto de despegue.
//
// DO_SET_HOME tiene dos formas:
//   param1 = 1                -> casa = donde está el dron AHORA
//   param1 = 0, param5/6/7    -> casa = esas coordenadas (lat, lon, alt AMSL)
// Este ejemplo usa la segunda: casa nueva a `north`/`east` metros de la
// original, a la altura del suelo.
//
// Flujo: arma (PX4 fija casa ahí) -> despega -> DO_SET_HOME -> espera a que
// home_position cambie -> Return -> aterriza en la casa nueva.
//
// OJO: PX4 publica home_position SOLO cuando cambia, y con best_effort un
// nodo que arranca después no recibe el último valor. Por eso no esperamos a
// tener casa antes de armar: al armar PX4 la vuelve a fijar y nos llega.
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples set_home
//   ros2 run px4_examples set_home --ros-args -p north:=10.0 -p east:=-5.0
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

//   WAIT_GPS -> ARMING -> TAKEOFF -> SET_HOME -> RETURN -> DONE
enum class State { WAIT_GPS, ARMING, TAKEOFF, SET_HOME, RETURN, DONE };

class SetHome : public rclcpp::Node
{
public:
  SetHome()
  : Node("set_home")
  {
    altitude_ = declare_parameter("altitude", 5.0);
    north_ = declare_parameter("north", 15.0);   // m de la casa original
    east_ = declare_parameter("east", 10.0);

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
    RCLCPP_INFO(get_logger(), "set_home: esperando posición GPS...");
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
          if (!home_.valid_hpos || !home_.valid_alt) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Todavía no llega la casa de PX4");
            break;
          }
          print_home("Casa actual (donde armó)");
          set_new_home();
          state_ = State::SET_HOME;
        }
        break;

      case State::SET_HOME:
        // update_count sube cada vez que PX4 cambia la casa: así sabemos que
        // ya aplicó la nueva (al armar también cambia, por eso lo guardamos
        // justo antes de mandar el comando).
        if (home_.update_count != home_count_) {
          print_home("Casa nueva");
          RCLCPP_INFO(get_logger(), "Return: debe aterrizar en la casa nueva");
          send_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
          state_ = State::RETURN;
        } else if (every_second()) {
          set_new_home();   // reintento
        }
        break;

      case State::RETURN:
        if (!armed) {
          // Distancia horizontal entre donde aterrizó y la casa nueva.
          const double error = std::hypot(home_.x - local_.x, home_.y - local_.y);
          RCLCPP_INFO(get_logger(), "Aterrizado a %.1f m de la casa nueva y desarmado", error);
          state_ = State::DONE;
        }
        break;

      case State::DONE:
        break;
    }
  }

  void set_new_home()
  {
    home_count_ = home_.update_count;
    const double lat = home_.lat + (north_ / kEarthRadius) * 180.0 / M_PI;
    const double lon = home_.lon +
      (east_ / (kEarthRadius * std::cos(home_.lat * M_PI / 180.0))) * 180.0 / M_PI;
    // param1 = 0: usar las coordenadas. param2..4 = roll/pitch/yaw de casa
    // (NaN en yaw = sin orientación). Altura: la misma de la casa original
    // (el suelo, en una cancha plana).
    send_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0.0f, 0.0f, 0.0f, kNaN, lat, lon,
                 home_.alt);
    RCLCPP_INFO(get_logger(), "DO_SET_HOME a %.0f m N, %.0f m E de la casa original",
                north_, east_);
  }

  void print_home(const char * label)
  {
    RCLCPP_INFO(get_logger(), "%s: %.7f, %.7f, %.1f m AMSL (local x=%.1f y=%.1f)", label,
                home_.lat, home_.lon, home_.alt, home_.x, home_.y);
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
  uint32_t home_count_ = 0;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetHome>());
  rclcpp::shutdown();
  return 0;
}
