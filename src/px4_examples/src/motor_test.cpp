// =============================================================================
// motor_test: hace girar UN motor (o todos, uno por uno) a cierto porcentaje
// durante unos segundos. Es lo mismo que la pestaña "Motores" de
// QGroundControl, pero desde ROS 2.
//
// Sirve para revisar en el dron real:
//   - que cada motor gire,
//   - que el motor 1 sea el que crees (el orden importa: si están cruzados el
//     dron se voltea al despegar),
//   - el sentido de giro de cada uno.
//
// Usa el comando ACTUATOR_TEST:
//   param1 = valor de 0 a 1 (0.15 = 15 % de empuje)
//   param2 = segundos; al terminar PX4 apaga el motor solo
//   param5 = qué salida: 1..8 = Motor 1..8 (101..108 también funciona)
//
// PX4 solo lo acepta con el dron DESARMADO y con COM_MOT_TEST_EN = 1 (por
// defecto). Si hay botón de seguridad, tiene que estar apagado (safety off).
//
//   ¡¡ SIN HÉLICES !!  Aunque esté desarmado, el motor SÍ gira.
//
// Ejecutar:
//   ros2 run px4_examples motor_test                                   # motor 1, 15 %, 2 s
//   ros2 run px4_examples motor_test --ros-args -p motor:=3 -p throttle:=0.2 -p seconds:=3.0
//   ros2 run px4_examples motor_test --ros-args -p motor:=0            # todos, en orden
// =============================================================================

#include <algorithm>   // std::clamp

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;

using namespace std::chrono_literals;

class MotorTest : public rclcpp::Node
{
public:
  MotorTest()
  : Node("motor_test")
  {
    motor_ = declare_parameter("motor", 1);            // 1..8, 0 = todos
    motor_count_ = declare_parameter("motor_count", 4);   // cuántos tiene el dron (para motor:=0)
    throttle_ = declare_parameter("throttle", 0.15);   // 0..1
    seconds_ = declare_parameter("seconds", 2.0);

    // Límite de seguridad: más de 30 % sin hélices no aporta nada y con
    // hélices sería peligroso.
    if (throttle_ > 0.3) {
      RCLCPP_WARN(get_logger(), "throttle %.2f es mucho para una prueba, lo bajo a 0.30", throttle_);
    }
    throttle_ = std::clamp(throttle_, 0.0, 0.3);
    if (motor_ < 0 || motor_ > 8) {
      RCLCPP_ERROR(get_logger(), "motor debe ser 1..8 (o 0 = todos)");
      motor_ = -1;   // no se probará nada
    }

    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg; have_status_ = true;});
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      [this](const VehicleCommandAck::SharedPtr msg) {
        if (msg->command != VehicleCommand::VEHICLE_CMD_ACTUATOR_TEST) {
          return;
        }
        if (msg->result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
          RCLCPP_INFO(get_logger(), "  PX4 aceptó la prueba");
        } else {
          // 2 = DENIED: armado, safety on o COM_MOT_TEST_EN = 0.
          RCLCPP_ERROR(get_logger(), "  PX4 rechazó la prueba (result=%u). "
                       "¿Está armado? ¿Safety on? ¿COM_MOT_TEST_EN = 0?", msg->result);
          current_ = last_;   // no seguir con los demás motores
        }
      });

    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    // first_ = primer motor a probar, last_ = último.
    first_ = motor_ == 0 ? 1 : motor_;
    last_ = motor_ == 0 ? motor_count_ : motor_;
    current_ = first_ - 1;   // "ninguno todavía"

    timer_ = create_wall_timer(100ms, [this]() {loop();});
  }

private:
  void loop()
  {
    if (motor_ < 0 || done_) {
      return;
    }
    if (!have_status_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Esperando a PX4...");
      return;
    }
    if (status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
      RCLCPP_ERROR(get_logger(), "El dron está ARMADO: la prueba de motores solo es desarmado");
      done_ = true;
      return;
    }

    // ¿Ya terminó el motor anterior? (dejamos 1 s de pausa entre motores)
    const bool idle = current_ < first_ || (now() - started_).seconds() > seconds_ + 1.0;
    if (!idle) {
      return;
    }
    if (current_ >= last_) {
      RCLCPP_INFO(get_logger(), "Prueba terminada");
      done_ = true;
      return;
    }

    current_++;
    RCLCPP_INFO(get_logger(), "Motor %ld al %.0f %% por %.1f s", current_, throttle_ * 100.0,
                seconds_);
    send_command(VehicleCommand::VEHICLE_CMD_ACTUATOR_TEST, static_cast<float>(throttle_),
                 static_cast<float>(seconds_), 0.0f, 0.0f, static_cast<double>(current_));
    started_ = now();
  }

  uint64_t now_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

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
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_;
  bool have_status_ = false;

  int64_t motor_, motor_count_;
  double throttle_, seconds_;

  int64_t first_, last_, current_;
  rclcpp::Time started_;
  bool done_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorTest>());
  rclcpp::shutdown();
  return 0;
}
