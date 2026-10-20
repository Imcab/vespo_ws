// =============================================================================
// @NODE@: solo LEE datos del dron, no manda nada a PX4.
//
// Se suscribe al estado (armado, modo), la posición local y la batería, e
// imprime un resumen una vez por segundo. Es seguro correrlo en cualquier
// momento, incluso con el dron volando con otro nodo.
//
// Ejecutar (con PX4 y el agente corriendo, p. ej. vespo up):
//   ros2 run @PKG@ @NODE@
// =============================================================================

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/battery_status.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::BatteryStatus;

using namespace std::chrono_literals;

class @CLASS@ : public rclcpp::Node
{
public:
  @CLASS@()
  : Node("@NODE@")
  {
    // Mismo QoS con el que publica PX4 (best_effort + transient_local).
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    // Cada callback solo guarda el último mensaje; el timer imprime, para no
    // llenar la terminal (algunos topics llegan a 50-100 Hz).
    // El sufijo _vN es la MESSAGE_VERSION del mensaje en px4_msgs.
    status_sub_ = create_subscription<VehicleStatus>(
      "@out:VehicleStatus@", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    local_sub_ = create_subscription<VehicleLocalPosition>(
      "@out:VehicleLocalPosition@", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});
    battery_sub_ = create_subscription<BatteryStatus>(
      "@out:BatteryStatus@", qos,
      [this](const BatteryStatus::SharedPtr msg) {battery_ = *msg;});

    timer_ = create_wall_timer(1s, [this]() {print_summary();});
    RCLCPP_INFO(get_logger(), "@NODE@ iniciado, esperando datos de PX4...");
  }

private:
  void print_summary()
  {
    // timestamp 0 = todavía no llegó ningún mensaje: suele ser que no corre
    // el agente o PX4, o que el nombre del topic no coincide.
    if (status_.timestamp == 0) {
      RCLCPP_WARN(get_logger(), "Sin datos de PX4 todavía (¿corre el agente? prueba: vespo up)");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "%s | nav=%d | x=%.2f y=%.2f alt=%.2f m | batería %.2f V (%.0f %%)",
      status_.arming_state == VehicleStatus::ARMING_STATE_ARMED ? "ARMADO" : "desarmado",
      status_.nav_state, local_.x, local_.y, -local_.z,   // NED: altura = -z
      battery_.voltage_v, battery_.remaining * 100.0f);
  }

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition local_;
  BatteryStatus battery_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<@CLASS@>());
  rclcpp::shutdown();
  return 0;
}
