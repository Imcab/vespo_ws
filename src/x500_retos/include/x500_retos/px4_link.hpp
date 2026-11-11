// =============================================================================
// px4_link.hpp: lo que TODAS las misiones del X500 repiten, en un solo lugar.
//
// En px4_examples cada nodo crea sus suscripciones, sus publishers y su
// send_command(); aquí eso ya lo vieron, así que vive en esta clase y cada
// misión se queda solo con su máquina de estados.
//
// Uso (dentro de un rclcpp::Node):
//
//   x500::Px4Link px4_{this};                 // miembro de la clase
//   px4_.local().x                            // posición NED (m)
//   px4_.armed(), px4_.in_offboard()          // estado
//   px4_.command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
//   px4_.offboard(setpoint);                  // heartbeat + setpoint, cada ciclo
//
// Topics: los de PX4 v1.17 (vehicle_status_v1, vehicle_local_position_v1,
// home_position_v1). Si PX4 cambia de versión y un mensaje sube de versión,
// solo hay que tocar este archivo.
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/home_position.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

namespace x500
{

using px4_msgs::msg::HomePosition;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleAttitude;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// NaN en un campo de un comando o setpoint = "no lo uses / no lo cambies".
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kEarthRadius = 6371000.0;   // m

// Un setpoint de offboard en NED (x = norte, y = este, z = abajo), en metros,
// m/s y m/s². Lo que se deja en NaN PX4 no lo usa.
struct Setpoint
{
  std::array<float, 3> position{kNaN, kNaN, kNaN};
  std::array<float, 3> velocity{kNaN, kNaN, kNaN};
  std::array<float, 3> acceleration{kNaN, kNaN, kNaN};
  float yaw = kNaN;        // rad, 0 = norte, positivo hacia el este
  float yawspeed = kNaN;   // rad/s, feedforward de giro (ayuda a que el yaw no se quede atrás)
};

// Ángulo a [-pi, pi]: para restar yaws sin que 359° - 1° dé 358°.
inline double wrap_pi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// (lat, lon) en grados a `north`, `east` metros de (lat, lon). Tierra plana:
// sobra para unos cuantos km. Ver goto_mission.cpp en px4_examples.
inline std::array<double, 2> gps_offset(double lat, double lon, double north, double east)
{
  return {
    lat + (north / kEarthRadius) * 180.0 / M_PI,
    lon + (east / (kEarthRadius * std::cos(lat * M_PI / 180.0))) * 180.0 / M_PI,
  };
}

// Distancia horizontal en metros entre dos puntos GPS (misma aproximación).
inline double gps_distance(double lat1, double lon1, double lat2, double lon2)
{
  const double north = (lat2 - lat1) * M_PI / 180.0 * kEarthRadius;
  const double east = (lon2 - lon1) * M_PI / 180.0 * kEarthRadius * std::cos(lat1 * M_PI / 180.0);
  return std::hypot(north, east);
}

class Px4Link
{
public:
  explicit Px4Link(rclcpp::Node * node)
  : node_(node)
  {
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    status_sub_ = node->create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    local_sub_ = node->create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});
    global_sub_ = node->create_subscription<VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos,
      [this](const VehicleGlobalPosition::SharedPtr msg) {global_ = *msg;});
    attitude_sub_ = node->create_subscription<VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      [this](const VehicleAttitude::SharedPtr msg) {attitude_ = *msg;});
    home_sub_ = node->create_subscription<HomePosition>(
      "/fmu/out/home_position_v1", qos,
      [this](const HomePosition::SharedPtr msg) {home_ = *msg;});
    ack_sub_ = node->create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      [this](const VehicleCommandAck::SharedPtr msg) {
        // Solo avisa de rechazos, una vez por cada respuesta distinta.
        const bool repeated = msg->command == last_ack_.command && msg->result == last_ack_.result;
        last_ack_ = *msg;
        if (!repeated && msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
          RCLCPP_WARN(node_->get_logger(), "PX4 rechazó el comando %u (result=%u)",
                      msg->command, msg->result);
        }
      });

    offboard_pub_ = node->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
    setpoint_pub_ = node->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
    command_pub_ = node->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);
  }

  // --- Lo que PX4 nos cuenta ------------------------------------------------
  const VehicleStatus & status() const {return status_;}
  const VehicleLocalPosition & local() const {return local_;}     // NED, origen donde arrancó PX4
  const VehicleGlobalPosition & global() const {return global_;}  // lat/lon en grados, alt AMSL
  const HomePosition & home() const {return home_;}               // "casa" (a donde va Return)
  // Actitud: q = {w, x, y, z}, del cuerpo (FRD) a NED.
  const VehicleAttitude & attitude() const {return attitude_;}

  bool armed() const {return status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;}
  bool in_offboard() const
  {
    return armed() && status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }
  bool in_hold() const {return status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;}
  bool local_ready() const {return local_.xy_valid && local_.z_valid;}
  bool gps_ready() const {return global_.lat_lon_valid && global_.alt_valid && local_.xy_valid;}

  // Distancia en 3D de la posición actual a un punto NED.
  float distance_to(const std::array<float, 3> & p) const
  {
    return std::sqrt((local_.x - p[0]) * (local_.x - p[0]) + (local_.y - p[1]) * (local_.y - p[1]) +
             (local_.z - p[2]) * (local_.z - p[2]));
  }

  // --- Comandos ---------------------------------------------------------------
  // param5/param6 son double porque ahí van latitud y longitud (un float no
  // alcanza: perdería ~1 m de precisión).
  void command(uint32_t cmd, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f,
    float p4 = 0.0f, double p5 = 0.0, double p6 = 0.0, float p7 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = cmd;
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

  // OFFBOARD + armar. Antes hay que haber mandado setpoints ~1 s: PX4 no
  // entra a offboard si no los está recibiendo ya.
  void request_offboard_and_arm()
  {
    command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // 6 = OFFBOARD
    command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  }

  // Hold (AUTO_LOITER). Desde Land (como queda al aterrizar) PX4 no deja
  // armar, así que antes de volver a armar hay que pasar por aquí.
  void request_hold() {command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 3.0f);}
  void arm() {command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);}
  void land() {command(VehicleCommand::VEHICLE_CMD_NAV_LAND);}
  void return_home() {command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);}

  // Heartbeat de offboard + setpoint. Hay que llamarlo en CADA ciclo mientras
  // el nodo controla: si PX4 deja de recibirlo (~0.5 s) entra en failsafe.
  void offboard(const Setpoint & sp)
  {
    OffboardControlMode mode{};
    // Basta un eje con valor: así se puede mezclar, p. ej. la altura por
    // posición (z) y el avance por velocidad (x, y en NaN).
    mode.position = any_finite(sp.position);
    mode.velocity = any_finite(sp.velocity);
    mode.acceleration = any_finite(sp.acceleration);
    mode.timestamp = now_us();
    offboard_pub_->publish(mode);

    TrajectorySetpoint msg{};
    msg.position = sp.position;
    msg.velocity = sp.velocity;
    msg.acceleration = sp.acceleration;
    msg.yaw = sp.yaw;
    msg.yawspeed = sp.yawspeed;
    msg.timestamp = now_us();
    setpoint_pub_->publish(msg);
  }

  uint64_t now_us() const {return node_->get_clock()->now().nanoseconds() / 1000;}

  static bool any_finite(const std::array<float, 3> & v)
  {
    return std::isfinite(v[0]) || std::isfinite(v[1]) || std::isfinite(v[2]);
  }

  // El reloj de PX4, en segundos (el timestamp de su posición). Para medir
  // tiempos de la misión conviene este y no now(): en SITL corre al paso de
  // la simulación, así que si Gazebo va más lento que el tiempo real la
  // trayectoria no se le adelanta al dron. now() es el reloj de la pared.
  double clock() const {return local_.timestamp * 1e-6;}

private:
  rclcpp::Node * node_;

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<HomePosition>::SharedPtr home_sub_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;

  VehicleStatus status_;
  VehicleLocalPosition local_;
  VehicleGlobalPosition global_;
  HomePosition home_;
  VehicleAttitude attitude_;
  VehicleCommandAck last_ack_;
};

}  // namespace x500
