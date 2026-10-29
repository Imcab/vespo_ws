// =============================================================================
// figure_eight: vuela un "8" en offboard siguiendo una trayectoria que es una
// función del TIEMPO, con feedforward de posición, velocidad y aceleración.
//
// Lo nuevo respecto a orbit_mission:
//   - La trayectoria p(t) es una fórmula; su derivada da la velocidad v(t) y
//     la segunda derivada la aceleración a(t). Mandar las tres hace que PX4
//     siga la curva casi sin retraso (sin feedforward se queda "atrás").
//   - El yaw sigue la dirección de avance: atan2(vy, vx). La nariz del dron
//     apunta siempre hacia donde se mueve, como un coche.
//
// La curva (lemniscata de Gerono), con w = 2*pi / period:
//     x(t) = A sin(w t)              (norte)
//     y(t) = A/2 sin(2 w t)          (este)
//   Pasa por el origen en t = 0, así que empieza justo donde despegó.
//
//          y (Este)
//          ▲
//     A/2  │     ╭──╮        ╭──╮
//          │    │    ╲    ╱    │
//       0  │────│─────╳──────│────► x (Norte)
//          │    │    ╱    ╲    │
//    -A/2  │     ╰──╯        ╰──╯
//               -A     0     A
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples figure_eight
//   ros2 run px4_examples figure_eight --ros-args -p size:=8.0 -p period:=25.0 -p laps:=3
//
// Velocidad máxima ~ size * 2*pi / period: con 5 m y 20 s, ~1.6 m/s.
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

enum class State { WARMUP, TAKEOFF, EIGHT, LANDING, DONE };

class FigureEight : public rclcpp::Node
{
public:
  FigureEight()
  : Node("figure_eight")
  {
    size_ = declare_parameter("size", 5.0);           // A: m de punta a centro
    period_ = declare_parameter("period", 20.0);      // s por cada 8
    altitude_ = declare_parameter("altitude", 5.0);   // m
    laps_ = declare_parameter("laps", 2);

    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {position_ = *msg;});

    offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    // 20 Hz: una trayectoria que cambia rápido se sigue mejor con más setpoints.
    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "figure_eight: %ld ochos de %.1f m en %.0f s cada uno",
                laps_, size_, period_);
  }

private:
  // Posición, velocidad y aceleración de la curva en el instante t.
  struct Sample
  {
    float x, y, vx, vy, ax, ay;
  };

  Sample curve(double t) const
  {
    const double w = 2.0 * M_PI / period_;
    const double a = size_;
    // Derivadas a mano: d/dt sin(w t) = w cos(w t), d/dt cos(w t) = -w sin(w t)
    return Sample{
      static_cast<float>(a * std::sin(w * t)),
      static_cast<float>(a / 2.0 * std::sin(2.0 * w * t)),
      static_cast<float>(a * w * std::cos(w * t)),
      static_cast<float>(a * w * std::cos(2.0 * w * t)),
      static_cast<float>(-a * w * w * std::sin(w * t)),
      static_cast<float>(-2.0 * a * w * w * std::sin(2.0 * w * t)),
    };
  }

  void loop()
  {
    // Valores por defecto: quieto en el origen, a la altura, mirando hacia
    // donde arranca la curva (así no gira de golpe al empezar el 8).
    Sample s = curve(0.0);
    s.vx = s.vy = s.ax = s.ay = 0.0f;
    float yaw = std::atan2(curve(0.0).vy, curve(0.0).vx);

    switch (state_) {
      case State::WARMUP:
        if (counter_ == 10) {
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // OFFBOARD
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        if (counter_ <= 10) {
          counter_++;
        }
        if (in_offboard()) {
          RCLCPP_INFO(get_logger(), "OFFBOARD activo, subiendo");
          state_ = State::TAKEOFF;
        }
        break;

      case State::TAKEOFF:
        if (std::hypot(position_.x, position_.y) < 0.3 &&
          std::abs(-altitude_ - position_.z) < 0.3)
        {
          RCLCPP_INFO(get_logger(), "A la altura, empezando el 8");
          start_ = now();
          state_ = State::EIGHT;
        }
        break;

      case State::EIGHT: {
          const double t = (now() - start_).seconds();
          if (t >= laps_ * period_) {
            // Tras N periodos la curva vuelve al origen: aterrizamos ahí.
            send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            RCLCPP_INFO(get_logger(), "%ld ochos completos, aterrizando", laps_);
            state_ = State::LANDING;
            return;
          }
          s = curve(t);
          yaw = std::atan2(s.vy, s.vx);
          break;
        }

      case State::LANDING:
        if (status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
          RCLCPP_INFO(get_logger(), "Aterrizado y desarmado");
          state_ = State::DONE;
        }
        return;

      case State::DONE:
        return;
    }

    if (state_ != State::WARMUP && !in_offboard()) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%d): abortado", status_.nav_state);
      state_ = State::DONE;
      return;
    }

    // position = true ya hace que PX4 use la velocidad y aceleración del
    // setpoint como feedforward; las activamos también para que quede claro.
    OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.velocity = true;
    offboard_msg.acceleration = true;
    offboard_msg.timestamp = now_us();
    offboard_pub_->publish(offboard_msg);

    TrajectorySetpoint setpoint_msg{};
    setpoint_msg.position = {s.x, s.y, static_cast<float>(-altitude_)};
    setpoint_msg.velocity = {s.vx, s.vy, 0.0f};
    setpoint_msg.acceleration = {s.ax, s.ay, 0.0f};
    setpoint_msg.yaw = yaw;
    setpoint_msg.timestamp = now_us();
    setpoint_pub_->publish(setpoint_msg);

    // Error de seguimiento: qué tan lejos está el dron de donde debería.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "nav=%d alt=%.2f m error=%.2f m", status_.nav_state, -position_.z,
      std::hypot(s.x - position_.x, s.y - position_.y));
  }

  bool in_offboard()
  {
    return status_.arming_state == VehicleStatus::ARMING_STATE_ARMED &&
           status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
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
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition position_;

  double size_, period_, altitude_;
  int64_t laps_;

  State state_ = State::WARMUP;
  rclcpp::Time start_;
  int counter_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FigureEight>());
  rclcpp::shutdown();
  return 0;
}
