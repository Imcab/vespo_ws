// =============================================================================
// @NODE@: arma, despega con el modo Takeoff de PX4, espera y aterriza.
//
// No usa OFFBOARD: solo manda comandos (VehicleCommand) y deja que los modos
// automáticos de PX4 (Takeoff, Hold, Land) hagan el vuelo. Es la forma más
// simple y segura de automatizar un vuelo.
//
// Máquina de estados (un paso cada 100 ms):
//   ESPERANDO_PX4 -> ARMANDO -> DESPEGANDO -> EN_HOVER -> ATERRIZANDO -> TERMINADO
//
// OJO: este nodo ARMA el vehículo. Pruébalo primero en simulación
// (vespo up --sim) y en el dron real sin hélices.
//
// Ejecutar (con PX4 y el agente corriendo, p. ej. vespo up):
//   ros2 run @PKG@ @NODE@
//   ros2 run @PKG@ @NODE@ --ros-args -p altitude:=3.0 -p hover_time:=5.0
// =============================================================================

#include <cmath>   // NAN

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleCommand;

using namespace std::chrono_literals;

class @CLASS@ : public rclcpp::Node
{
public:
  @CLASS@()
  : Node("@NODE@")
  {
    altitude_ = declare_parameter("altitude", 5.0);       // m sobre el punto de despegue
    hover_time_ = declare_parameter("hover_time", 5.0);   // s en el aire antes de aterrizar

    // PX4 publica con best_effort + transient_local; el QoS por defecto
    // (reliable) no recibe nada y no da ningún error.
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    // El sufijo _vN es la MESSAGE_VERSION del mensaje en px4_msgs.
    status_sub_ = create_subscription<VehicleStatus>(
      "@out:VehicleStatus@", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});
    local_sub_ = create_subscription<VehicleLocalPosition>(
      "@out:VehicleLocalPosition@", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});
    global_sub_ = create_subscription<VehicleGlobalPosition>(
      "@out:VehicleGlobalPosition@", qos,
      [this](const VehicleGlobalPosition::SharedPtr msg) {global_ = *msg;});
    command_pub_ = create_publisher<VehicleCommand>("@in:VehicleCommand@", qos);

    timer_ = create_wall_timer(100ms, [this]() {step();});
    RCLCPP_INFO(get_logger(), "@NODE@: despegue a %.1f m, hover %.0f s y aterrizaje", altitude_, hover_time_);
  }

private:
  enum class State { WaitingPx4, Arming, TakingOff, Hovering, Landing, Done };

  void step()
  {
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    const double alt = -local_.z;   // NED: altura = -z
    const double in_state_s = (now() - state_since_).seconds();

    switch (state_) {
      case State::WaitingPx4:
        // Hace falta posición válida para despegar (y la global para la altura AMSL)
        if (status_.timestamp != 0 && local_.z_valid && global_.timestamp != 0) {
          change(State::Arming, "PX4 listo, armando");
        }
        break;

      case State::Arming:
        // Reintenta cada segundo: el primer comando puede perderse (best_effort)
        if (armed) {
          // NAV_TAKEOFF pide la altura en AMSL (sobre el nivel del mar):
          // altura actual del suelo + la que queremos subir.
          takeoff_amsl_ = global_.alt + altitude_;
          change(State::TakingOff, "Armado, despegando");
        } else if (every_second()) {
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        break;

      case State::TakingOff:
        if (!armed) {
          change(State::Done, "Se desarmó antes de despegar (¿COM_DISARM_PRFLT?)");
        } else if (alt > altitude_ - 0.3) {
          change(State::Hovering, "Altura alcanzada");
        } else if (status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF && every_second()) {
          // param4 = yaw (NAN: el actual), param7 = altitud AMSL
          send_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN,
                       static_cast<float>(takeoff_amsl_));
        }
        break;

      case State::Hovering:
        // Tras el despegue PX4 pasa solo a Hold: aquí solo esperamos
        if (in_state_s > hover_time_) {
          change(State::Landing, "Aterrizando");
        }
        break;

      case State::Landing:
        if (!armed) {
          change(State::Done, "En tierra y desarmado. Fin");
        } else if (status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LAND && every_second()) {
          send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        }
        break;

      case State::Done:
        timer_->cancel();
        rclcpp::shutdown();   // spin() retorna y el programa termina
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s | nav=%d | alt=%.2f m",
                         armed ? "ARMADO" : "desarmado", status_.nav_state, alt);
  }

  void change(State next, const char * why)
  {
    state_ = next;
    state_since_ = now();
    last_command_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "%s", why);
  }

  // true como mucho una vez por segundo: para reintentar comandos sin saturar
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

  void send_command(uint32_t command, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f,
                    float p4 = 0.0f, float p7 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.param3 = p3;
    msg.param4 = p4;
    msg.param5 = NAN;   // latitud: NAN = la actual
    msg.param6 = NAN;   // longitud: NAN = la actual
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
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  VehicleLocalPosition local_;
  VehicleGlobalPosition global_;

  double altitude_;
  double hover_time_;
  double takeoff_amsl_ = 0.0;
  State state_ = State::WaitingPx4;
  rclcpp::Time state_since_{now()};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<@CLASS@>());
  rclcpp::shutdown();
  return 0;
}
