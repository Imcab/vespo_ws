// =============================================================================
// battery_monitor: lee la batería y avisa cuando baja de un porcentaje.
// Opcionalmente le pide a PX4 regresar a casa (RTL) o aterrizar.
//
// Lee /fmu/out/battery_status_v1 (PX4 lo publica a 1 Hz):
//   voltage_v         voltaje total (V)
//   current_a         corriente (A); -1 = sin sensor de corriente (como en SITL)
//   remaining         carga restante de 0 a 1 (-1 = desconocida)
//   time_remaining_s  tiempo estimado hasta vacía con el consumo actual (s)
//   cell_count        número de celdas (4S = 4)
//   warning           aviso de PX4: 0 ninguno, 1 bajo, 2 crítico, 3 emergencia
//
// PX4 ya tiene su propio failsafe de batería (BAT_LOW_THR, BAT_CRIT_THR,
// COM_LOW_BAT_ACT). Este nodo sirve para avisar ANTES, con tu propio umbral,
// o para que tu misión decida qué hacer (p. ej. cortar la misión y volver).
//
// Ejecutar:
//   ros2 run px4_examples battery_monitor
//   ros2 run px4_examples battery_monitor --ros-args -p low_percent:=40.0 -p action:=rtl
//
// action: "none" (solo avisa), "rtl" (regresa a casa) o "land" (aterriza ahí).
// Solo actúa si el dron está armado: en tierra no tiene sentido.
// =============================================================================

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleCommand;

class BatteryMonitor : public rclcpp::Node
{
public:
  BatteryMonitor()
  : Node("battery_monitor")
  {
    low_percent_ = declare_parameter("low_percent", 30.0);
    action_ = declare_parameter("action", std::string("none"));
    if (action_ != "none" && action_ != "rtl" && action_ != "land") {
      RCLCPP_WARN(get_logger(), "action '%s' no existe, uso 'none'", action_.c_str());
      action_ = "none";
    }

    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    // Aquí no hay timer: todo pasa en el callback, cada vez que llega un
    // mensaje de batería (1 por segundo).
    battery_sub_ = create_subscription<BatteryStatus>(
      "/fmu/out/battery_status_v1", qos,
      [this](const BatteryStatus::SharedPtr msg) {on_battery(*msg);});

    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    RCLCPP_INFO(get_logger(), "battery_monitor: aviso bajo %.0f %%, acción '%s'",
                low_percent_, action_.c_str());
  }

private:
  void on_battery(const BatteryStatus & bat)
  {
    if (!bat.connected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Sin batería conectada");
      return;
    }

    // remaining viene de 0 a 1; -1 si PX4 todavía no la estima.
    const float percent = bat.remaining * 100.0f;
    const float cell_v = bat.cell_count > 0 ? bat.voltage_v / bat.cell_count : 0.0f;

    // std::isfinite: time_remaining_s es NaN cuando no hay estimación.
    if (std::isfinite(bat.time_remaining_s)) {
      RCLCPP_INFO(get_logger(), "%.1f %% | %.2f V (%.2f V/celda, %dS) | %.1f A | quedan %.0f s | %s",
                  percent, bat.voltage_v, cell_v, bat.cell_count, bat.current_a,
                  bat.time_remaining_s, warning_name(bat.warning));
    } else {
      RCLCPP_INFO(get_logger(), "%.1f %% | %.2f V (%.2f V/celda, %dS) | %.1f A | %s",
                  percent, bat.voltage_v, cell_v, bat.cell_count, bat.current_a,
                  warning_name(bat.warning));
    }

    if (bat.remaining < 0.0f || percent >= low_percent_) {
      return;
    }

    // De aquí para abajo: batería baja. Avisamos y actuamos UNA sola vez.
    if (!warned_) {
      RCLCPP_WARN(get_logger(), "BATERÍA BAJA: %.1f %% (umbral %.0f %%)", percent, low_percent_);
      warned_ = true;
    }
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    if (acted_ || !armed || action_ == "none") {
      return;
    }
    if (action_ == "rtl") {
      send_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
      RCLCPP_WARN(get_logger(), "Regresando a casa (RTL)");
    } else {
      send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
      RCLCPP_WARN(get_logger(), "Aterrizando aquí (LAND)");
    }
    acted_ = true;
  }

  // const char* = texto fijo de C, lo que pide el %s de RCLCPP_INFO.
  static const char * warning_name(uint8_t warning)
  {
    switch (warning) {
      case BatteryStatus::WARNING_NONE: return "OK";
      case BatteryStatus::WARNING_LOW: return "PX4: BAJA";
      case BatteryStatus::WARNING_CRITICAL: return "PX4: CRÍTICA";
      case BatteryStatus::WARNING_EMERGENCY: return "PX4: EMERGENCIA";
      case BatteryStatus::WARNING_FAILED: return "PX4: FALLÓ";
      default: return "PX4: ?";
    }
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
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;

  VehicleStatus status_;

  double low_percent_;
  std::string action_;
  bool warned_ = false;   // ya avisamos de batería baja
  bool acted_ = false;    // ya mandamos RTL / LAND
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitor>());
  rclcpp::shutdown();
  return 0;
}
