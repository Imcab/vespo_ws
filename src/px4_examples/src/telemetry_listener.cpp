// =============================================================================
// telemetry_listener: solo LEE datos del dron (no manda nada a PX4).
//
// Se suscribe a:
//   /fmu/out/vehicle_status_v1           -> armado y modo de vuelo
//   /fmu/out/vehicle_local_position_v1   -> posición/velocidad local (NED, metros)
//   /fmu/out/vehicle_global_position     -> latitud, longitud, altitud GPS
//   /fmu/out/vehicle_attitude            -> orientación (cuaternión -> roll/pitch/yaw)
//   /fmu/out/battery_status_v1           -> voltaje y porcentaje de batería
//
// Imprime un resumen una vez por segundo. Es seguro correrlo en cualquier
// momento, incluso con el dron volando con otro nodo.
//
// Ejecutar:
//   ros2 run px4_examples telemetry_listener
// =============================================================================

#include <algorithm>   // std::clamp
#include <cmath>       // std::atan2, std::asin, M_PI (funciones matemáticas)

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/battery_status.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleAttitude;
using px4_msgs::msg::BatteryStatus;

using namespace std::chrono_literals;

class TelemetryListener : public rclcpp::Node
{
public:
  TelemetryListener()
  : Node("telemetry_listener")
  {
    // Mismo QoS que PX4 usa al publicar (best_effort + transient_local).
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    // Cada callback solo guarda la copia más reciente del mensaje.
    // La impresión la hace el timer, para no llenar la terminal
    // (algunos topics llegan a 50-100 Hz).

    // Sufijo _v4: VehicleStatus tiene MESSAGE_VERSION = 4.
    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});

    // Sufijo _v1: VehicleLocalPosition tiene MESSAGE_VERSION = 1.
    local_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});

    // Sin sufijo: VehicleGlobalPosition tiene MESSAGE_VERSION = 0.
    global_sub_ = create_subscription<VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos,
      [this](const VehicleGlobalPosition::SharedPtr msg) {global_ = *msg;});

    // Sin sufijo: VehicleAttitude tiene MESSAGE_VERSION = 0.
    attitude_sub_ = create_subscription<VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      [this](const VehicleAttitude::SharedPtr msg) {attitude_ = *msg;});

    // Sufijo _v1: BatteryStatus tiene MESSAGE_VERSION = 1.
    battery_sub_ = create_subscription<BatteryStatus>(
      "/fmu/out/battery_status_v1", qos,
      [this](const BatteryStatus::SharedPtr msg) {battery_ = *msg;});

    // Imprime el resumen cada segundo.
    timer_ = create_wall_timer(1s, [this]() {print_summary();});

    RCLCPP_INFO(get_logger(), "telemetry_listener iniciado, esperando datos de PX4...");
  }

private:
  void print_summary()
  {
    // Si timestamp sigue en 0 es que todavía no ha llegado ningún mensaje
    // (los mensajes se crean con todos sus campos en 0). Suele indicar que
    // PX4 o el agente no están corriendo, o que el nombre del topic está mal.
    if (status_.timestamp == 0) {
      RCLCPP_WARN(get_logger(), "Sin datos de vehicle_status todavía (¿corre el agente?)");
      return;   // salimos de la función sin imprimir el resto
    }

    // --- Orientación: cuaternión -> ángulos de Euler --------------------
    // PX4 da el cuaternión en orden [w, x, y, z] (ROS usa [x, y, z, w]).
    const float w = attitude_.q[0];
    const float x = attitude_.q[1];
    const float y = attitude_.q[2];
    const float z = attitude_.q[3];
    // Fórmulas estándar de cuaternión a roll/pitch/yaw (resultado en radianes).
    const float roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    // asin solo acepta valores en [-1, 1]; std::clamp evita NaN por redondeo.
    const float pitch = std::asin(std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f));
    const float yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

    // Separador para leer cada bloque con facilidad.
    RCLCPP_INFO(get_logger(), "------------------------------------------------");

    // --- Estado ------------------------------------------------------------
    // "? :" es el operador ternario: condición ? si_verdadero : si_falso
    // (en Python: "ARMADO" if condición else "desarmado").
    // %s imprime texto (const char*).
    RCLCPP_INFO(
      get_logger(), "Estado:   %s | modo: %s (%d)",
      status_.arming_state == VehicleStatus::ARMING_STATE_ARMED ? "ARMADO" : "desarmado",
      nav_state_name(status_.nav_state), status_.nav_state);

    // --- Posición local (NED) ---------------------------------------------
    // x = norte, y = este, z = abajo. Altura = -z.
    RCLCPP_INFO(
      get_logger(), "Local:    x=%.2f y=%.2f alt=%.2f m | vel=(%.2f, %.2f, %.2f) m/s %s",
      local_.x, local_.y, -local_.z, local_.vx, local_.vy, local_.vz,
      local_.xy_valid ? "" : "[xy NO válido]");

    // --- Posición global (GPS) --------------------------------------------
    // lat/lon son double (float64); %.7f da ~1 cm de resolución.
    RCLCPP_INFO(
      get_logger(), "GPS:      lat=%.7f lon=%.7f alt=%.1f m (AMSL)",
      global_.lat, global_.lon, global_.alt);

    // --- Orientación en grados ----------------------------------------------
    // rad -> grados: multiplicar por 180 / pi.
    RCLCPP_INFO(
      get_logger(), "Actitud:  roll=%.1f° pitch=%.1f° yaw=%.1f°",
      rad2deg(roll), rad2deg(pitch), rad2deg(yaw));

    // --- Batería -------------------------------------------------------------
    // remaining va de 0 a 1 (o -1 si es inválido); lo pasamos a porcentaje.
    RCLCPP_INFO(
      get_logger(), "Batería:  %.2f V | %.0f %%",   // "%%" imprime un "%" literal
      battery_.voltage_v, battery_.remaining * 100.0f);
  }

  // Convierte radianes a grados. "static" = no usa atributos del nodo.
  static float rad2deg(float rad)
  {
    return rad * 180.0f / static_cast<float>(M_PI);
  }

  // Traduce el número de nav_state a un nombre legible.
  // "switch" es como "match" en Python; cada case hace "return", así que no
  // hace falta "break". Uso las constantes del mensaje en vez de números.
  static const char * nav_state_name(uint8_t nav_state)
  {
    switch (nav_state) {
      case VehicleStatus::NAVIGATION_STATE_MANUAL: return "MANUAL";
      case VehicleStatus::NAVIGATION_STATE_ALTCTL: return "ALTITUDE";
      case VehicleStatus::NAVIGATION_STATE_POSCTL: return "POSITION";
      case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "MISSION";
      case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: return "HOLD";
      case VehicleStatus::NAVIGATION_STATE_AUTO_RTL: return "RTL";
      case VehicleStatus::NAVIGATION_STATE_ACRO: return "ACRO";
      case VehicleStatus::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";
      case VehicleStatus::NAVIGATION_STATE_STAB: return "STABILIZED";
      case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "TAKEOFF";
      case VehicleStatus::NAVIGATION_STATE_AUTO_LAND: return "LAND";
      default: return "OTRO";   // cualquier otro valor
    }
  }

  // Suscripciones (hay que guardarlas o se destruyen).
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Último mensaje recibido de cada topic.
  VehicleStatus status_;
  VehicleLocalPosition local_;
  VehicleGlobalPosition global_;
  VehicleAttitude attitude_;
  BatteryStatus battery_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryListener>());
  rclcpp::shutdown();
  return 0;
}
