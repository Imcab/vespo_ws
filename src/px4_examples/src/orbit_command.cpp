// =============================================================================
// orbit_command: despega, da vueltas alrededor de un punto y regresa a casa
// SIN modo offboard: solo le mandamos COMANDOS a PX4 y él vuela todo.
//
// Es la otra forma de controlar el dron. En offboard (orbit_mission.cpp)
// nosotros calculamos cada setpoint 10 veces por segundo; aquí le decimos a
// PX4 "despega", "orbita aquí", "regresa", y sus modos automáticos hacen el
// resto. Menos control fino, pero mucho menos código y más seguro: si el nodo
// se cae, PX4 sigue en su modo y no hay failsafe de offboard.
//
// Comandos que usa (VehicleCommand, los mismos de MAVLink):
//   COMPONENT_ARM_DISARM  armar
//   NAV_TAKEOFF           modo Takeoff de PX4 hasta una altitud
//   DO_ORBIT              modo Orbit de PX4: radio, velocidad y centro
//   NAV_RETURN_TO_LAUNCH  modo Return: vuelve a casa y aterriza
//
// También lee /fmu/out/vehicle_command_ack: PX4 responde a cada comando si lo
// aceptó o lo rechazó (y así sabemos por qué no hizo caso).
//
// Ejecutar (con PX4 SITL y el agente corriendo, p. ej. vespo up --sim):
//   ros2 run px4_examples orbit_command
//   ros2 run px4_examples orbit_command --ros-args -p radius:=10.0 -p speed:=3.0 -p laps:=1.5
// =============================================================================

#include <cmath>    // M_PI, std::abs
#include <limits>   // std::numeric_limits<float>::quiet_NaN()

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;

using namespace std::chrono_literals;

// NaN ("not a number") en un parámetro de comando significa "no lo cambies /
// usa el valor actual". PX4 lo revisa con PX4_ISFINITE().
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Radio de la Tierra para pasar metros a grados de latitud.
constexpr double kEarthRadius = 6371000.0;   // m

//   WAIT_GPS -> ARMING -> TAKEOFF -> ORBIT -> RETURN -> DONE
enum class State { WAIT_GPS, ARMING, TAKEOFF, ORBIT, RETURN, DONE };

class OrbitCommand : public rclcpp::Node
{
public:
  OrbitCommand()
  : Node("orbit_command")
  {
    altitude_ = declare_parameter("altitude", 5.0);   // m sobre el suelo
    radius_ = declare_parameter("radius", 5.0);       // m (MC_ORBIT_RAD_MAX limita el máximo)
    speed_ = declare_parameter("speed", 2.0);         // m/s; negativo = sentido antihorario
    laps_ = declare_parameter("laps", 2.0);           // puede ser fracción: 1.5 vueltas

    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    local_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});
    // Posición GLOBAL: latitud, longitud (grados) y altitud sobre el nivel del
    // mar (AMSL). Los comandos de PX4 usan coordenadas globales, no NED.
    global_sub_ = create_subscription<VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos,
      [this](const VehicleGlobalPosition::SharedPtr msg) {global_ = *msg;});
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      [this](const VehicleCommandAck::SharedPtr msg) {on_ack(*msg);});

    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    // Aquí no hay heartbeat de offboard: 10 Hz solo para revisar el estado.
    timer_ = create_wall_timer(100ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "orbit_command: esperando posición GPS...");
  }

private:
  void loop()
  {
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;

    switch (state_) {
      case State::WAIT_GPS:
        // Sin posición global válida PX4 rechaza Takeoff y Orbit.
        if (global_.lat_lon_valid && global_.alt_valid) {
          RCLCPP_INFO(get_logger(), "GPS listo (%.7f, %.7f, %.1f m AMSL), armando",
                      global_.lat, global_.lon, global_.alt);
          home_z_ = local_.z;   // z del suelo, para medir la altura del despegue
          state_ = State::ARMING;
        }
        break;

      case State::ARMING:
        if (!armed) {
          // Reintenta una vez por segundo (el primero puede perderse).
          if (every_second()) {
          // Primero a Hold (AUTO_LOITER): en algunos modos, como Land (el modo
          // en que queda tras aterrizar), PX4 no deja armar.
          // DO_SET_MODE: param1 = 1 (modo custom), param2 = 4 (AUTO), param3 = 3 (LOITER).
            if (status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
              send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 3.0f);
            } else {
              send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
            }
          }
          break;
        }
        // Takeoff: param4 = yaw (NaN = el actual), param5/6 = lat/lon,
        // param7 = altitud AMSL. Por eso sumamos la altura deseada a la altitud
        // actual sobre el nivel del mar.
        send_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, kNaN, kNaN, kNaN, kNaN,
                     global_.lat, global_.lon, global_.alt + static_cast<float>(altitude_));
        RCLCPP_INFO(get_logger(), "Armado, despegando a %.1f m", altitude_);
        state_ = State::TAKEOFF;
        break;

      case State::TAKEOFF:
        // Al terminar el despegue PX4 se queda en Hold (AUTO_LOITER).
        if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER &&
          std::abs(home_z_ - local_.z - altitude_) < 0.5)
        {
          start_orbit();
          state_ = State::ORBIT;
        }
        break;

      case State::ORBIT: {
          const double elapsed = (now() - orbit_start_).seconds();
          // Tiempo de una vuelta = perímetro / velocidad = 2*pi*r / v.
          const double lap_time = 2.0 * M_PI * radius_ / std::abs(speed_);
          if (elapsed > laps_ * lap_time) {
            send_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
            RCLCPP_INFO(get_logger(), "%.1f vueltas, regresando a casa", laps_);
            state_ = State::RETURN;
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Orbitando: vuelta %.2f de %.1f",
                               elapsed / lap_time, laps_);
          break;
        }

      case State::RETURN:
        // RTL aterriza y PX4 desarma solo al tocar tierra.
        if (!armed) {
          RCLCPP_INFO(get_logger(), "En casa y desarmado");
          state_ = State::DONE;
        }
        break;

      case State::DONE:
        break;
    }
  }

  void start_orbit()
  {
    // Ponemos el centro `radius` metros al NORTE del dron, así ya empieza
    // sobre el círculo. 1 grado de latitud = R * pi / 180 metros (~111 km),
    // así que d metros al norte son d / R radianes = d / R * 180 / pi grados.
    const double center_lat = global_.lat + (radius_ / kEarthRadius) * 180.0 / M_PI;

    // DO_ORBIT: param1 = radio, param2 = velocidad, param3 = hacia dónde mira
    // (0 = al centro), param5/6 = centro (lat/lon), param7 = altitud (NaN =
    // la actual).
    send_command(VehicleCommand::VEHICLE_CMD_DO_ORBIT,
                 static_cast<float>(radius_), static_cast<float>(speed_),
                 VehicleCommand::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER, kNaN,
                 center_lat, global_.lon, kNaN);
    orbit_start_ = now();
    RCLCPP_INFO(get_logger(), "Altura alcanzada, orbitando %.1f m de radio a %.1f m/s",
                radius_, speed_);
  }

  void on_ack(const VehicleCommandAck & ack)
  {
    // Solo imprime cuando cambia la respuesta, no cada reintento.
    if (ack.command == last_ack_.command && ack.result == last_ack_.result) {
      return;
    }
    last_ack_ = ack;
    if (ack.result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
      RCLCPP_INFO(get_logger(), "PX4 aceptó el comando %u", ack.command);
    } else {
      // result: 1 = temporalmente rechazado, 2 = denegado, 3 = no soportado,
      // 4 = falló. El motivo exacto sale en la consola de PX4 / QGroundControl.
      RCLCPP_WARN(get_logger(), "PX4 rechazó el comando %u (result=%u)", ack.command, ack.result);
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

  // Igual que en los otros ejemplos pero con los 7 parámetros. param5 y
  // param6 son double porque llevan latitud y longitud (los float solo tienen
  // ~7 cifras, y 19.4326077 ya son 9: perderíamos precisión de metros).
  void send_command(uint32_t command, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f,
    float p4 = 0.0f, double p5 = 0.0, double p6 = 0.0, float p7 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
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

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition local_;
  VehicleGlobalPosition global_;
  VehicleCommandAck last_ack_;

  double altitude_, radius_, speed_, laps_;
  float home_z_ = 0.0f;

  State state_ = State::WAIT_GPS;
  rclcpp::Time orbit_start_;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbitCommand>());
  rclcpp::shutdown();
  return 0;
}
