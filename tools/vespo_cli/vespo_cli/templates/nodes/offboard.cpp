// =============================================================================
// @NODE@: despega en modo OFFBOARD y se mantiene en hover a `altitude` metros.
//
// Flujo:
//   1. Durante ~1 s publica heartbeat (OffboardControlMode) + setpoint
//      (TrajectorySetpoint): PX4 rechaza entrar a OFFBOARD si no le llegan
//      setpoints ANTES de pedirlo.
//   2. En el ciclo 10 (1 s) manda los comandos OFFBOARD y ARM, una sola vez.
//   3. Sigue publicando heartbeat + setpoint. Si se dejan de publicar
//      (p. ej. Ctrl+C), PX4 activa el failsafe (COM_OBL_RC_ACT).
//
// OJO: este nodo ARMA el vehículo. Pruébalo primero en simulación
// (vespo up --sim) y en el dron real sin hélices.
//
// Ejecutar (con PX4 y el agente corriendo, p. ej. vespo up):
//   ros2 run @PKG@ @NODE@
//   ros2 run @PKG@ @NODE@ --ros-args -p altitude:=3.0
// =============================================================================

#include "rclcpp/rclcpp.hpp"

// Mensajes de PX4. El archivo va en snake_case aunque el tipo sea CamelCase.
#include "px4_msgs/msg/vehicle_status.hpp"           // armado, modo de vuelo
#include "px4_msgs/msg/vehicle_local_position.hpp"   // posición x, y, z en NED
#include "px4_msgs/msg/offboard_control_mode.hpp"    // heartbeat: qué controlamos
#include "px4_msgs/msg/trajectory_setpoint.hpp"      // a dónde queremos ir
#include "px4_msgs/msg/vehicle_command.hpp"          // armar, cambiar de modo, aterrizar

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
    // Parámetro ROS: altura de hover en metros (cámbiala con -p altitude:=3.0).
    altitude_ = declare_parameter("altitude", 5.0);

    // PX4 publica con best_effort + transient_local. Con el QoS por defecto
    // (reliable) la suscripción no recibe nada y no da ningún error.
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    // Topics: el sufijo _vN es la MESSAGE_VERSION del mensaje en px4_msgs
    // (vespo new lo calculó al crear el paquete). Si actualizas PX4 y deja de
    // llegar algo, compáralos con: ros2 topic list | grep fmu
    status_sub_ = create_subscription<VehicleStatus>(
      "@out:VehicleStatus@", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    position_sub_ = create_subscription<VehicleLocalPosition>(
      "@out:VehicleLocalPosition@", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {position_ = *msg;});

    offboard_pub_ = create_publisher<OffboardControlMode>("@in:OffboardControlMode@", qos);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("@in:TrajectorySetpoint@", qos);
    command_pub_ = create_publisher<VehicleCommand>("@in:VehicleCommand@", qos);

    // 10 Hz: PX4 exige el heartbeat a más de 2 Hz.
    timer_ = create_wall_timer(100ms, [this]() {loop();});

    RCLCPP_INFO(get_logger(), "@NODE@ iniciado, hover a %.1f m. Calentando setpoints...", altitude_);
  }

private:
  void loop()
  {
    // 1) Heartbeat: "sigo vivo y controlo por POSICIÓN".
    OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.timestamp = now_us();
    offboard_pub_->publish(offboard_msg);

    // 2) Setpoint en NED (x norte, y este, z ABAJO): z negativa = arriba.
    TrajectorySetpoint setpoint_msg{};
    setpoint_msg.position = {0.0f, 0.0f, static_cast<float>(-altitude_)};
    setpoint_msg.yaw = 0.0f;   // rad; 0 = mirando al norte
    setpoint_msg.timestamp = now_us();
    setpoint_pub_->publish(setpoint_msg);

    // 3) Tras ~1 s de setpoints: OFFBOARD y ARM, una sola vez.
    if (counter_ == 10) {
      // DO_SET_MODE: param1 = 1 (modo custom de PX4), param2 = 6 (OFFBOARD).
      send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      RCLCPP_INFO(get_logger(), "Comandos OFFBOARD + ARM enviados");
    }
    if (counter_ <= 10) {
      counter_++;
    }

    // 4) Log, como mucho una vez por segundo.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "%s | nav=%d | alt=%.2f m",
      status_.arming_state == VehicleStatus::ARMING_STATE_ARMED ? "ARMADO" : "desarmado",
      status_.nav_state, -position_.z);
  }

  // Tiempo en microsegundos, la unidad de los timestamps de PX4.
  uint64_t now_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  // Publica un VehicleCommand (el equivalente a un comando MAVLink).
  void send_command(uint32_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.target_system = 1;      // el dron
    msg.target_component = 1;   // el autopiloto
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;   // viene de fuera de PX4 (de ROS 2)
    msg.timestamp = now_us();
    command_pub_->publish(msg);
  }

  // Suscripciones, publishers y timer deben ser atributos: si fueran locales
  // del constructor se destruirían al salir de él.
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition position_;
  double altitude_;
  int counter_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<@CLASS@>());
  rclcpp::shutdown();
  return 0;
}
