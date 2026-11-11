// =============================================================================
// vuelo_prueba: el "piloto travieso" para probar la geocerca (misión 4).
// Despega y se lanza en línea recta, sin frenar, hasta que alguien le quite el
// control (el guardián manda Hold) o hasta alejarse `max_distance` m (ahí se
// detiene solo: es una red de seguridad por si el guardián no funciona).
//
// Mundo:   vespo up --sim --world plaza_concierto
// Correr:  ros2 run x500_retos guardian_geocerca          # terminal 2
//          ros2 run x500_retos vuelo_prueba                # terminal 3
//          ros2 run x500_retos vuelo_prueba --ros-args -p direction_deg:=90.0 -p speed:=6.0
//
// direction_deg: hacia dónde se escapa, 0 = norte (al escenario), 90 = este.
// Este nodo NO es un reto: ya está completo. También se puede volar a mano
// desde QGroundControl en vez de usarlo.
// =============================================================================

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using x500::kNaN;
using x500::Setpoint;

//   WARMUP -> TAKEOFF -> DASH -> STOPPED (alguien tomó el control o llegó al tope)
enum class State { WARMUP, TAKEOFF, DASH, STOPPED };

class VueloPrueba : public rclcpp::Node
{
public:
  VueloPrueba()
  : Node("vuelo_prueba")
  {
    direction_ = declare_parameter("direction_deg", 0.0) * M_PI / 180.0;
    speed_ = declare_parameter("speed", 4.0);               // m/s
    altitude_ = declare_parameter("altitude", 3.0);         // m
    max_distance_ = declare_parameter("max_distance", 30.0);   // m, tope de seguridad

    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "vuelo_prueba: me voy a escapar hacia %.0f° a %.1f m/s",
                direction_ * 180.0 / M_PI, speed_);
  }

private:
  void loop()
  {
    const float z = static_cast<float>(-altitude_);
    switch (state_) {
      case State::WARMUP: {
          Setpoint sp;
          sp.position = {0.0f, 0.0f, z};
          sp.yaw = static_cast<float>(direction_);
          px4_.offboard(sp);
          // Tras ~1 s de setpoints pedir OFFBOARD y armar; y reintentar cada
        // segundo por si PX4 todavía no estaba listo (o lo rechazó).
        if (++counter_ % 20 == 0) {
            px4_.request_offboard_and_arm();
          }
          if (px4_.in_offboard()) {
            state_ = State::TAKEOFF;
          }
          break;
        }

      case State::TAKEOFF: {
          Setpoint sp;
          sp.position = {0.0f, 0.0f, z};
          sp.yaw = static_cast<float>(direction_);
          px4_.offboard(sp);
          if (px4_.distance_to(sp.position) < 0.3f) {
            RCLCPP_INFO(get_logger(), "Arriba. ¡Ahí voy!");
            state_ = State::DASH;
          }
          break;
        }

      case State::DASH: {
          if (!px4_.in_offboard()) {
            RCLCPP_WARN(get_logger(), "Me quitaron el control (nav=%u) a %.1f m del despegue",
                        px4_.status().nav_state, std::hypot(px4_.local().x, px4_.local().y));
            state_ = State::STOPPED;
            break;
          }
          if (std::hypot(px4_.local().x, px4_.local().y) > max_distance_) {
            RCLCPP_ERROR(get_logger(), "Nadie me detuvo en %.0f m: me paro solo y aterrizo", max_distance_);
            px4_.land();
            state_ = State::STOPPED;
            break;
          }
          // Velocidad horizontal hacia `direction_`; la altura, por posición.
          Setpoint sp;
          sp.position = {kNaN, kNaN, z};
          sp.velocity = {static_cast<float>(speed_ * std::cos(direction_)),
            static_cast<float>(speed_ * std::sin(direction_)), kNaN};
          sp.yaw = static_cast<float>(direction_);
          px4_.offboard(sp);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "a %.1f m del despegue",
                               std::hypot(px4_.local().x, px4_.local().y));
          break;
        }

      case State::STOPPED:
        // Ya no mandamos nada: PX4 (o quien tomó el control) decide.
        rclcpp::shutdown();
        break;
    }
  }

  x500::Px4Link px4_{this};
  rclcpp::TimerBase::SharedPtr timer_;
  double direction_, speed_, altitude_, max_distance_;
  State state_ = State::WARMUP;
  int counter_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VueloPrueba>());
  rclcpp::shutdown();
  return 0;
}
