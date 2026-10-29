// =============================================================================
// change_speed: cambia la velocidad del dron EN PLENO VUELO con el comando
// DO_CHANGE_SPEED, y muestra la velocidad medida para ver el cambio.
//
// Flujo:
//   1. Despega (Takeoff de PX4) y va hacia un punto a `distance` m al norte
//      a `slow` m/s (DO_REPOSITION con param1 = velocidad).
//   2. A un tercio del camino manda DO_CHANGE_SPEED a `fast` m/s.
//   3. Al llegar regresa a casa (Return).
//
// DO_CHANGE_SPEED: param1 = tipo (1 = velocidad sobre el suelo),
//                  param2 = velocidad en m/s (-1 = volver a la de por defecto),
//                  param3 = acelerador en % (-1 = no cambiar).
// Afecta a los modos automáticos de PX4 (Hold/Go to, Mission). En offboard
// no hace nada: ahí la velocidad la decide tu setpoint.
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples change_speed
//   ros2 run px4_examples change_speed --ros-args -p slow:=1.0 -p fast:=8.0 -p distance:=60.0
// =============================================================================

#include <cmath>
#include <limits>

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

//   WAIT_GPS -> ARMING -> TAKEOFF -> SLOW -> FAST -> RETURN -> DONE
enum class State { WAIT_GPS, ARMING, TAKEOFF, SLOW, FAST, RETURN, DONE };

class ChangeSpeed : public rclcpp::Node
{
public:
  ChangeSpeed()
  : Node("change_speed")
  {
    altitude_ = declare_parameter("altitude", 5.0);
    distance_ = declare_parameter("distance", 45.0);   // m al norte
    slow_ = declare_parameter("slow", 2.0);            // m/s
    fast_ = declare_parameter("fast", 6.0);            // m/s (máximo: MPC_XY_VEL_MAX)

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
        const bool repeated = msg->command == last_ack_.command && msg->result == last_ack_.result;
        last_ack_ = *msg;
        if (!repeated && msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
          RCLCPP_WARN(get_logger(), "PX4 rechazó el comando %u (result=%u)",
                      msg->command, msg->result);
        }
      });

    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    timer_ = create_wall_timer(100ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "change_speed: esperando posición GPS...");
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
          start_x_ = local_.x;
          start_y_ = local_.y;
          const double lat = global_.lat + (distance_ / kEarthRadius) * 180.0 / M_PI;
          send_command(VehicleCommand::VEHICLE_CMD_DO_REPOSITION, static_cast<float>(slow_), 1.0f,
                       0.0f, kNaN, lat, global_.lon, global_.alt);
          RCLCPP_INFO(get_logger(), "Hacia %.0f m al norte a %.1f m/s", distance_, slow_);
          state_ = State::SLOW;
        }
        break;

      case State::SLOW:
        report();
        if (traveled() > distance_ / 3.0) {
          // param1 = 1: velocidad sobre el suelo. param3 = -1: no tocar el acelerador.
          send_command(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 1.0f,
                       static_cast<float>(fast_), -1.0f);
          RCLCPP_INFO(get_logger(), "DO_CHANGE_SPEED a %.1f m/s", fast_);
          state_ = State::FAST;
        }
        break;

      case State::FAST:
        report();
        if (traveled() > distance_ - 1.0) {
          RCLCPP_INFO(get_logger(), "Llegué, regresando a casa");
          send_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
          state_ = State::RETURN;
        }
        break;

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

  // Velocidad horizontal medida: |(vx, vy)| de la posición local.
  void report()
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "recorrido %4.1f m | velocidad %.1f m/s",
                         traveled(), std::hypot(local_.vx, local_.vy));
  }

  double traveled()
  {
    return std::hypot(local_.x - start_x_, local_.y - start_y_);
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
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition local_;
  VehicleGlobalPosition global_;
  VehicleCommandAck last_ack_;

  double altitude_, distance_, slow_, fast_;

  State state_ = State::WAIT_GPS;
  float ground_z_ = 0.0f, start_x_ = 0.0f, start_y_ = 0.0f;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChangeSpeed>());
  rclcpp::shutdown();
  return 0;
}
