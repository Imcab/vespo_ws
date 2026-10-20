// =============================================================================
// @NODE@: vuela en círculo alrededor de un punto, mirando al centro.
//
// Muestra cómo combinar setpoints de POSICIÓN con VELOCIDAD de feedforward:
// la posición dice dónde estar en el círculo y la velocidad (tangente al
// círculo) le dice a PX4 cómo moverse, así no va "a tirones" de punto en punto.
//
// Flujo:
//   1. ~1 s de heartbeat + setpoint, luego OFFBOARD + ARM.
//   2. Sube y va al punto de inicio del círculo (al norte del centro).
//   3. Da `laps` vueltas a `speed` m/s y aterriza (LAND de PX4).
//
// OJO: este nodo ARMA el vehículo. Pruébalo primero en simulación
// (vespo up --sim) y en el dron real sin hélices.
//
// Ejecutar (con PX4 y el agente corriendo, p. ej. vespo up):
//   ros2 run @PKG@ @NODE@
//   ros2 run @PKG@ @NODE@ --ros-args -p radius:=8.0 -p speed:=3.0 -p laps:=3
// =============================================================================

#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

using namespace std::chrono_literals;

class @CLASS@ : public rclcpp::Node
{
public:
  @CLASS@()
  : Node("@NODE@")
  {
    center_x_ = declare_parameter("center_x", 0.0);   // m al norte del origen
    center_y_ = declare_parameter("center_y", 0.0);   // m al este del origen
    altitude_ = declare_parameter("altitude", 5.0);   // m
    radius_ = declare_parameter("radius", 5.0);       // m
    speed_ = declare_parameter("speed", 2.0);         // m/s sobre el círculo
    laps_ = declare_parameter("laps", 2);

    auto qos = rclcpp::QoS(1).best_effort().transient_local();
    // El sufijo _vN es la MESSAGE_VERSION del mensaje en px4_msgs.
    status_sub_ = create_subscription<VehicleStatus>(
      "@out:VehicleStatus@", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    local_sub_ = create_subscription<VehicleLocalPosition>(
      "@out:VehicleLocalPosition@", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});
    offboard_pub_ = create_publisher<OffboardControlMode>("@in:OffboardControlMode@", qos);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("@in:TrajectorySetpoint@", qos);
    command_pub_ = create_publisher<VehicleCommand>("@in:VehicleCommand@", qos);

    timer_ = create_wall_timer(100ms, [this]() {step();});   // dt = 0.1 s
    RCLCPP_INFO(get_logger(), "@NODE@: %ld vueltas de %.1f m de radio a %.1f m/s",
                laps_, radius_, speed_);
  }

private:
  enum class State { WarmUp, Approach, Orbiting, Landing };
  static constexpr double kDt = 0.1;

  void step()
  {
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;

    if (state_ == State::Landing) {
      if (!armed) {
        RCLCPP_INFO(get_logger(), "En tierra y desarmado. Fin");
        timer_->cancel();
        rclcpp::shutdown();
      } else if (every_second() && status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
        send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
      }
      return;
    }

    // Punto del círculo para el ángulo actual (0 = al norte del centro)
    const double x = center_x_ + radius_ * std::cos(angle_);
    const double y = center_y_ + radius_ * std::sin(angle_);
    double vx = 0.0, vy = 0.0;

    switch (state_) {
      case State::WarmUp:
        if (++ticks_ < 10) {
          break;
        }
        if (armed && status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          RCLCPP_INFO(get_logger(), "En OFFBOARD, yendo al inicio del círculo");
          state_ = State::Approach;
        } else if (every_second()) {
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // 6 = OFFBOARD
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        break;

      case State::Approach:
        if (std::hypot(x - local_.x, y - local_.y) < 0.3 && std::abs(-altitude_ - local_.z) < 0.3) {
          RCLCPP_INFO(get_logger(), "En el círculo, orbitando");
          state_ = State::Orbiting;
        }
        break;

      case State::Orbiting: {
          // Velocidad angular = v / r. La velocidad lineal es tangente al círculo.
          const double omega = speed_ / radius_;
          vx = -speed_ * std::sin(angle_);
          vy = speed_ * std::cos(angle_);
          angle_ += omega * kDt;
          if (angle_ >= 2.0 * M_PI * laps_) {
            RCLCPP_INFO(get_logger(), "%ld vueltas completas, aterrizando", laps_);
            state_ = State::Landing;
            return;
          }
          break;
        }

      default:
        break;
    }

    // Heartbeat: posición + velocidad (feedforward)
    OffboardControlMode offboard{};
    offboard.position = true;
    offboard.velocity = true;
    offboard.timestamp = now_us();
    offboard_pub_->publish(offboard);

    TrajectorySetpoint setpoint{};
    setpoint.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(-altitude_)};
    setpoint.velocity = {static_cast<float>(vx), static_cast<float>(vy), 0.0f};
    // Mirando al centro: el ángulo del vector dron -> centro
    setpoint.yaw = static_cast<float>(std::atan2(center_y_ - y, center_x_ - x));
    setpoint.timestamp = now_us();
    setpoint_pub_->publish(setpoint);

    if (state_ != State::WarmUp && status_.nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%d): abortado", status_.nav_state);
      timer_->cancel();
      rclcpp::shutdown();
    }
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

  void send_command(uint32_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
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
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition local_;

  double center_x_, center_y_, altitude_, radius_, speed_;
  int64_t laps_;
  State state_ = State::WarmUp;
  double angle_ = 0.0;
  int ticks_ = 0;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<@CLASS@>());
  rclcpp::shutdown();
  return 0;
}
