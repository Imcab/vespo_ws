// =============================================================================
// orbit_mission: vuela en círculo alrededor de un punto, mirando al centro,
// calculando NOSOTROS la trayectoria en modo OFFBOARD.
//
// Lo nuevo respecto a takeoff_mission / square_mission:
//   - Setpoint de POSICIÓN + VELOCIDAD (feedforward). La posición dice dónde
//     estar en el círculo y la velocidad (tangente al círculo) le dice a PX4
//     cómo moverse, así no va "a tirones" de punto en punto.
//   - Parámetros de ROS 2 (radio, velocidad, vueltas...) que se cambian al
//     ejecutar, sin recompilar.
//   - Si alguien saca al dron de OFFBOARD (p. ej. el piloto con el control),
//     el nodo deja de mandar setpoints y termina.
//
// Flujo (máquina de estados):
//   WARMUP   -> ~1 s de setpoints, luego OFFBOARD + ARM
//   APPROACH -> sube y va al punto de inicio del círculo (al norte del centro)
//   ORBIT    -> da `laps` vueltas a `speed` m/s
//   LANDING  -> LAND de PX4 hasta que desarma
//
// Para que PX4 vuele el círculo por su cuenta (sin calcularlo nosotros) ver
// orbit_command.cpp.
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples orbit_mission
//   ros2 run px4_examples orbit_mission --ros-args -p radius:=8.0 -p speed:=3.0 -p laps:=3
// =============================================================================

#include <cmath>    // std::cos, std::sin, std::atan2, std::hypot, M_PI

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

enum class State { WARMUP, APPROACH, ORBIT, LANDING, DONE };

class OrbitMission : public rclcpp::Node
{
public:
  OrbitMission()
  : Node("orbit_mission")
  {
    // declare_parameter(nombre, valor por defecto) registra el parámetro y
    // devuelve su valor. Se cambia con --ros-args -p radius:=8.0
    // El tipo sale del valor por defecto: 5.0 -> double, 2 -> int64.
    center_x_ = declare_parameter("center_x", 0.0);   // m al norte del origen
    center_y_ = declare_parameter("center_y", 0.0);   // m al este del origen
    altitude_ = declare_parameter("altitude", 5.0);   // m sobre el origen
    radius_ = declare_parameter("radius", 5.0);       // m
    speed_ = declare_parameter("speed", 2.0);         // m/s sobre el círculo
    laps_ = declare_parameter("laps", 2);             // vueltas

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

    timer_ = create_wall_timer(100ms, [this]() {loop();});   // dt = 0.1 s

    RCLCPP_INFO(get_logger(), "orbit_mission: %ld vueltas de %.1f m de radio a %.1f m/s",
                laps_, radius_, speed_);
  }

private:
  static constexpr double kDt = 0.1;   // periodo del timer en segundos

  void loop()
  {
    // Punto del círculo para el ángulo actual. angle_ = 0 -> al norte del
    // centro, y crece en sentido horario visto desde arriba (norte -> este).
    const double x = center_x_ + radius_ * std::cos(angle_);
    const double y = center_y_ + radius_ * std::sin(angle_);
    double vx = 0.0;
    double vy = 0.0;

    switch (state_) {
      case State::WARMUP:
        if (counter_ == 10) {
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // 6 = OFFBOARD
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        if (counter_ <= 10) {
          counter_++;
        }
        if (in_offboard()) {
          RCLCPP_INFO(get_logger(), "OFFBOARD activo, yendo al inicio del círculo");
          state_ = State::APPROACH;
        }
        break;

      case State::APPROACH:
        // std::hypot(a, b) = sqrt(a*a + b*b): distancia horizontal.
        if (std::hypot(x - position_.x, y - position_.y) < 0.3 &&
          std::abs(-altitude_ - position_.z) < 0.3)
        {
          RCLCPP_INFO(get_logger(), "En el círculo, orbitando");
          state_ = State::ORBIT;
        }
        break;

      case State::ORBIT: {
          // Las llaves {} crean un bloque: hacen falta para declarar variables
          // (omega) dentro de un case.
          //
          // Velocidad angular omega = v / r (rad/s). Cada ciclo avanzamos
          // omega * dt radianes sobre el círculo.
          const double omega = speed_ / radius_;
          // Derivada de (r cos a, r sin a): la velocidad es tangente al círculo.
          vx = -speed_ * std::sin(angle_);
          vy = speed_ * std::cos(angle_);
          angle_ += omega * kDt;

          if (angle_ >= 2.0 * M_PI * laps_) {
            send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            RCLCPP_INFO(get_logger(), "%ld vueltas completas, aterrizando", laps_);
            state_ = State::LANDING;
          }
          break;
        }

      case State::LANDING:
        if (status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
          RCLCPP_INFO(get_logger(), "Aterrizado y desarmado");
          state_ = State::DONE;
        }
        return;   // en LANDING ya no mandamos setpoints: PX4 está en AUTO_LAND

      case State::DONE:
        return;
    }

    // Si el piloto (o un failsafe) cambió el modo, dejamos de controlar.
    if (state_ != State::WARMUP && !in_offboard()) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%d): misión abortada", status_.nav_state);
      state_ = State::DONE;
      return;
    }

    // Heartbeat: esta vez activamos posición Y velocidad.
    OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.velocity = true;
    offboard_msg.timestamp = now_us();
    offboard_pub_->publish(offboard_msg);

    // static_cast<float>(x) convierte double -> float (los campos son float).
    TrajectorySetpoint setpoint_msg{};
    setpoint_msg.position = {static_cast<float>(x), static_cast<float>(y),
      static_cast<float>(-altitude_)};
    setpoint_msg.velocity = {static_cast<float>(vx), static_cast<float>(vy), 0.0f};
    // Yaw mirando al centro: el ángulo del vector dron -> centro.
    // atan2(este, norte) da el yaw en NED (0 = norte, positivo hacia el este).
    setpoint_msg.yaw = static_cast<float>(std::atan2(center_y_ - y, center_x_ - x));
    setpoint_msg.timestamp = now_us();
    setpoint_pub_->publish(setpoint_msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "nav=%d alt=%.2f m vuelta=%.2f", status_.nav_state, -position_.z,
      angle_ / (2.0 * M_PI));
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

  // Parámetros
  double center_x_, center_y_, altitude_, radius_, speed_;
  int64_t laps_;   // los parámetros enteros de ROS 2 son int64

  State state_ = State::WARMUP;
  double angle_ = 0.0;   // rad recorridos sobre el círculo
  int counter_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbitMission>());
  rclcpp::shutdown();
  return 0;
}
