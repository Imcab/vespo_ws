// =============================================================================
// @NODE@: misión OFFBOARD por una lista de waypoints y aterrizaje.
//
// Los waypoints son un parámetro ROS: una lista plana [x0, y0, alt0, x1, ...]
// en metros, con x = norte, y = este respecto al punto donde arrancó PX4
// (ROS 2 no tiene "lista de listas": se aplana y se recorre de 3 en 3).
//
// Flujo:
//   1. ~1 s de heartbeat + setpoint antes de pedir OFFBOARD (PX4 lo exige).
//   2. OFFBOARD + ARM.
//   3. Va a cada waypoint mirando hacia donde avanza; al llegar (a menos de
//      `tolerance` m) espera `hold_time` s y pasa al siguiente.
//   4. Al terminar pide LAND (modo automático de PX4) y sale al desarmar.
//
// OJO: este nodo ARMA el vehículo. Pruébalo primero en simulación
// (vespo up --sim) y en el dron real sin hélices.
//
// Ejecutar (con PX4 y el agente corriendo, p. ej. vespo up):
//   ros2 run @PKG@ @NODE@
//   ros2 run @PKG@ @NODE@ --ros-args -p "waypoints:=[0.0, 0.0, 3.0, 4.0, 0.0, 3.0]"
// =============================================================================

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

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
    // Por defecto: un cuadrado de 5 m a 5 m de altura que vuelve al inicio
    waypoints_ = declare_parameter(
      "waypoints", std::vector<double>{0, 0, 5, 5, 0, 5, 5, 5, 5, 0, 5, 5, 0, 0, 5});
    tolerance_ = declare_parameter("tolerance", 0.3);   // m
    hold_time_ = declare_parameter("hold_time", 1.0);   // s en cada waypoint

    // Mejor fallar al arrancar que en el aire
    if (waypoints_.empty() || waypoints_.size() % 3 != 0) {
      RCLCPP_ERROR(get_logger(), "waypoints debe tener x, y, alt por punto (múltiplo de 3), tiene %zu valores",
                   waypoints_.size());
      throw std::invalid_argument("waypoints");
    }

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

    timer_ = create_wall_timer(100ms, [this]() {step();});
    RCLCPP_INFO(get_logger(), "@NODE@: %zu waypoints", count());
  }

private:
  enum class State { WarmUp, Flying, Holding, Landing, Done };

  size_t count() const {return waypoints_.size() / 3;}

  // Waypoint i en NED (z hacia abajo, por eso -alt)
  std::array<float, 3> target(size_t i) const
  {
    return {static_cast<float>(waypoints_[3 * i]), static_cast<float>(waypoints_[3 * i + 1]),
      static_cast<float>(-waypoints_[3 * i + 2])};
  }

  void step()
  {
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;

    if (state_ == State::Landing || state_ == State::Done) {
      // En LAND manda PX4: ya no publicamos setpoints
      if (state_ == State::Landing && !armed) {
        RCLCPP_INFO(get_logger(), "En tierra y desarmado. Fin");
        state_ = State::Done;
      }
      if (state_ == State::Done) {
        timer_->cancel();
        rclcpp::shutdown();
        return;
      }
      if (every_second() && status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
        send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
      }
      return;
    }

    publish_setpoint(target(current_));

    switch (state_) {
      case State::WarmUp:
        // Tras ~1 s de setpoints, OFFBOARD y ARM; reintenta cada segundo
        if (++ticks_ < 10) {
          break;
        }
        if (armed && status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          RCLCPP_INFO(get_logger(), "En OFFBOARD, hacia el waypoint 1/%zu", count());
          state_ = State::Flying;
        } else if (every_second()) {
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // 6 = OFFBOARD
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        break;

      case State::Flying:
        if (distance_to(target(current_)) < tolerance_) {
          RCLCPP_INFO(get_logger(), "Waypoint %zu/%zu alcanzado", current_ + 1, count());
          state_ = State::Holding;
          hold_since_ = now();
        }
        break;

      case State::Holding:
        if ((now() - hold_since_).seconds() >= hold_time_) {
          if (++current_ == count()) {
            RCLCPP_INFO(get_logger(), "Misión completa, aterrizando");
            state_ = State::Landing;
          } else {
            RCLCPP_INFO(get_logger(), "Hacia el waypoint %zu/%zu", current_ + 1, count());
            state_ = State::Flying;
          }
        }
        break;

      default:
        break;
    }

    // Si algo (RC, failsafe, QGC) saca al dron de OFFBOARD, abandonamos
    if (state_ != State::WarmUp && status_.nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%d): misión abortada", status_.nav_state);
      timer_->cancel();
      rclcpp::shutdown();
    }
  }

  void publish_setpoint(const std::array<float, 3> & position)
  {
    OffboardControlMode offboard{};
    offboard.position = true;
    offboard.timestamp = now_us();
    offboard_pub_->publish(offboard);

    // Mira hacia el waypoint mientras está lejos; cerca conserva el último yaw
    const float dx = position[0] - local_.x;
    const float dy = position[1] - local_.y;
    if (std::hypot(dx, dy) > 1.0f) {
      yaw_ = std::atan2(dy, dx);
    }

    TrajectorySetpoint setpoint{};
    setpoint.position = position;
    setpoint.yaw = yaw_;
    setpoint.timestamp = now_us();
    setpoint_pub_->publish(setpoint);
  }

  double distance_to(const std::array<float, 3> & p) const
  {
    return std::sqrt(std::pow(p[0] - local_.x, 2) + std::pow(p[1] - local_.y, 2) + std::pow(p[2] - local_.z, 2));
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

  std::vector<double> waypoints_;
  double tolerance_;
  double hold_time_;
  State state_ = State::WarmUp;
  size_t current_ = 0;
  int ticks_ = 0;
  float yaw_ = 0.0f;
  rclcpp::Time hold_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<@CLASS@>());
  rclcpp::shutdown();
  return 0;
}
