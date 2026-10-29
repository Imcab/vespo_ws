// =============================================================================
// velocity_control: controla el dron por VELOCIDAD en offboard, no por
// posición. Es como manejarlo con un joystick desde el código: "sube a 1 m/s",
// "avanza a 2 m/s", "gira a 0.5 rad/s"... durante cierto tiempo.
//
// Lo nuevo respecto a takeoff_mission:
//   - OffboardControlMode con velocity = true (y position = false).
//   - En TrajectorySetpoint la posición va en NaN: le dice a PX4 "no controles
//     posición, solo velocidad". Lo mismo con yaw = NaN y yawspeed = valor.
//   - Velocidad en el marco del CUERPO (adelante / derecha del dron) pasada a
//     NED con el heading actual. Es lo que se usa para seguir algo con la
//     cámara: "avanza hacia donde miras".
//   - El despegue se cierra con la ALTURA medida, no con el tiempo: PX4 tarda
//     unos segundos en arrancar motores y hacer la rampa de despegue, así que
//     "sube 5 s a 1 m/s" no da 5 m (se queda casi en el suelo).
//
// OJO: en velocidad el dron NO se queda en un punto. Si mandas 0 se frena,
// pero con viento se va moviendo poco a poco (no corrige posición).
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples velocity_control
// =============================================================================

#include <cmath>    // std::cos, std::sin
#include <limits>   // NaN
#include <string>
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

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Un "tramo" del vuelo: qué velocidad mandar y por cuánto tiempo.
// struct = una clase con todo público, para agrupar datos (como un dataclass).
struct Leg
{
  std::string name;
  float forward;    // m/s; en NED = norte, en cuerpo = hacia el frente del dron
  float right;      // m/s; en NED = este, en cuerpo = hacia la derecha del dron
  float down;       // m/s; negativo = subir
  float yawspeed;   // rad/s; positivo = girar a la derecha (horario visto desde arriba)
  double seconds;
  bool body_frame;  // true = forward/right relativos a hacia donde mira el dron
};

enum class State { WARMUP, TAKEOFF, FLYING, LANDING, DONE };

class VelocityControl : public rclcpp::Node
{
public:
  VelocityControl()
  : Node("velocity_control")
  {
    altitude_ = declare_parameter("altitude", 5.0);   // m a los que sube antes del plan

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

    timer_ = create_wall_timer(100ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "velocity_control iniciado, %zu tramos", legs_.size());
  }

private:
  // El plan de vuelo, después del despegue. Cambia, quita o agrega tramos.
  //   nombre                        fwd   right down  yawspeed  s    cuerpo
  std::vector<Leg> legs_ = {
    {"norte a 2 m/s",                2.0f, 0.0f, 0.0f, 0.0f,    4.0, false},
    {"este a 2 m/s",                 0.0f, 2.0f, 0.0f, 0.0f,    4.0, false},
    {"girar en el lugar",            0.0f, 0.0f, 0.0f, 0.5f,    6.0, false},
    {"adelante (hacia donde mira)",  2.0f, 0.0f, 0.0f, 0.0f,    4.0, true},
    {"adelante girando (curva)",     2.0f, 0.0f, 0.0f, 0.4f,    6.0, true},
    {"bajar a 0.5 m/s",              0.0f, 0.0f, 0.5f, 0.0f,    3.0, false},
    {"frenar",                       0.0f, 0.0f, 0.0f, 0.0f,    2.0, false},
  };

  void loop()
  {
    if (state_ == State::LANDING) {
      if (status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
        RCLCPP_INFO(get_logger(), "Aterrizado y desarmado");
        state_ = State::DONE;
      }
      return;
    }
    if (state_ == State::DONE) {
      return;
    }

    // Por defecto (WARMUP) velocidad 0: armado en el suelo no se mueve.
    Leg leg{"quieto", 0.0f, 0.0f, 0.0f, 0.0f, 0.0, false};

    if (state_ == State::WARMUP) {
      if (counter_ == 10) {
        send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // OFFBOARD
        send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      }
      if (counter_ <= 10) {
        counter_++;
      }
      if (in_offboard()) {
        RCLCPP_INFO(get_logger(), "OFFBOARD activo, subiendo a %.1f m", altitude_);
        state_ = State::TAKEOFF;
      }
    } else if (!in_offboard()) {   // TAKEOFF o FLYING y alguien cambió el modo
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%d): abortado", status_.nav_state);
      state_ = State::DONE;
      return;
    } else if (state_ == State::TAKEOFF) {
      // Subir a 1 m/s hasta que la altura MEDIDA llegue al objetivo.
      leg = Leg{"despegue", 0.0f, 0.0f, -1.0f, 0.0f, 0.0, false};
      if (-position_.z > altitude_ - 0.2) {
        start_leg(0);
        state_ = State::FLYING;
      }
    } else {   // FLYING
      if ((now() - leg_start_).seconds() > legs_[leg_index_].seconds) {
        if (leg_index_ + 1 == legs_.size()) {
          send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
          RCLCPP_INFO(get_logger(), "Plan terminado, aterrizando");
          state_ = State::LANDING;
          return;
        }
        start_leg(leg_index_ + 1);
      }
      leg = legs_[leg_index_];
    }

    // --- Velocidad en NED ---------------------------------------------------
    float vx = leg.forward;
    float vy = leg.right;
    if (leg.body_frame) {
      // Rotar (adelante, derecha) por el heading (yaw) del dron:
      //   norte = adelante*cos(yaw) - derecha*sin(yaw)
      //   este  = adelante*sin(yaw) + derecha*cos(yaw)
      // heading = 0 mirando al norte, pi/2 mirando al este.
      const float yaw = position_.heading;
      vx = leg.forward * std::cos(yaw) - leg.right * std::sin(yaw);
      vy = leg.forward * std::sin(yaw) + leg.right * std::cos(yaw);
    }

    OffboardControlMode offboard_msg{};
    offboard_msg.position = false;
    offboard_msg.velocity = true;
    offboard_msg.timestamp = now_us();
    offboard_pub_->publish(offboard_msg);

    TrajectorySetpoint setpoint_msg{};
    // NaN = "esto no lo controles". Si dejáramos {0, 0, 0} PX4 intentaría
    // también volver al origen.
    setpoint_msg.position = {kNaN, kNaN, kNaN};
    setpoint_msg.velocity = {vx, vy, leg.down};
    setpoint_msg.yaw = kNaN;
    setpoint_msg.yawspeed = leg.yawspeed;
    setpoint_msg.timestamp = now_us();
    setpoint_pub_->publish(setpoint_msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "%s: v=(%.1f, %.1f, %.1f) m/s alt=%.2f m", leg.name.c_str(),
      position_.vx, position_.vy, position_.vz, -position_.z);
  }

  void start_leg(size_t index)
  {
    leg_index_ = index;
    leg_start_ = now();
    RCLCPP_INFO(get_logger(), "Tramo %zu: %s (%.0f s)", index, legs_[index].name.c_str(),
                legs_[index].seconds);
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

  double altitude_;

  State state_ = State::WARMUP;
  size_t leg_index_ = 0;
  rclcpp::Time leg_start_;
  int counter_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityControl>());
  rclcpp::shutdown();
  return 0;
}
