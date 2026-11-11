// =============================================================================
// cruzar_cueva (misión 5): atraviesa una cueva curva y oscura usando SOLO el
// lidar 2D para no chocar: se centra entre las paredes, gira hacia donde hay
// espacio y esquiva la columna de roca que hay a medio camino.
//
// Mundo:   vespo up --sim --world cueva --custom x500_lidar_2d
// Correr:  ros2 launch x500_retos cueva.launch.py                 # puente del lidar + nodo
//          ros2 launch x500_retos cueva.launch.py paquete:=x500_retos
//
// Viene de: velocity_control (offboard por velocidad, marco del cuerpo).
// Lo nuevo: control REACTIVO. No hay waypoints adentro: cada 50 ms se mira el
// scan y se decide la velocidad. El puente ros_gz_bridge (en el launch) pasa
// el lidar de Gazebo a ROS como sensor_msgs/LaserScan en /x500/scan.
//
// El scan, visto desde arriba (ángulos del lidar, positivo = izquierda):
//
//                      0° (frente)
//               +45°    |    -45°
//                  \    |    /
//        +90° ------  dron  ------ -90°      cubre de -135° a +135°
//        (izq)                    (der)
//
// Flujo:  WARMUP -> TAKEOFF -> TO_ENTRANCE (por posición, hasta la boca)
//         -> CAVE (con el lidar) -> TO_PAD (sale y va a la plataforma)
//         -> LANDING -> DONE
//
// Seguridad (ya hecha): si hay algo a menos de stop_distance al frente, no
// avanza; si pasa stuck_time segundos sin moverse dentro de la cueva, aterriza.
//
// Criterio de éxito: sale por la boca norte sin tocar nada (lo más cerca que
// pasó el lidar de la roca > 0.6 m) y aterriza en la plataforma naranja.
//
// TU PARTE: el TODO en steer(). Mientras tanto el dron entra derecho y
// despacio, se detiene frente a la primera pared y aterriza por "atorado".
// =============================================================================

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::LaserScan;
using x500::kNaN;
using x500::Setpoint;

enum class State { WARMUP, TAKEOFF, TO_ENTRANCE, CAVE, TO_PAD, LANDING, DONE };

// Lo que decide steer(): velocidades en el marco del dron.
struct Command
{
  double forward = 0.0;   // m/s hacia el frente
  double left = 0.0;      // m/s hacia la izquierda
  double yawrate = 0.0;   // rad/s, positivo = girar a la izquierda (como los ángulos del scan)
};

class CruzarCueva : public rclcpp::Node
{
public:
  CruzarCueva()
  : Node("cruzar_cueva")
  {
    altitude_ = declare_parameter("altitude", 1.5);          // m
    max_speed_ = declare_parameter("max_speed", 1.5);        // m/s
    stop_distance_ = declare_parameter("stop_distance", 1.0);  // m al frente para no avanzar
    entrance_n_ = declare_parameter("entrance_north", 11.0);  // 1 m adentro de la boca sur, NED
    entrance_e_ = declare_parameter("entrance_east", 0.0);
    exit_n_ = declare_parameter("exit_north", 78.0);         // ya afuera por el norte
    pad_n_ = declare_parameter("pad_north", 84.0);           // plataforma de salida
    pad_e_ = declare_parameter("pad_east", -1.0);
    stuck_time_ = declare_parameter("stuck_time", 15.0);     // s
    const auto topic = declare_parameter("scan_topic", std::string("/x500/scan"));

    scan_sub_ = create_subscription<LaserScan>(
      topic, rclcpp::SensorDataQoS(),
      [this](const LaserScan::SharedPtr msg) {scan_ = msg;});

    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "cruzar_cueva: esperando el lidar en %s", topic.c_str());
  }

private:
  // ---------------------------------------------------------------------------
  // La distancia más corta que ve el lidar entre dos ángulos (en grados,
  // positivo = izquierda). Lo que no pega en nada cuenta como range_max.
  // ---------------------------------------------------------------------------
  double sector_min(double from_deg, double to_deg) const
  {
    double best = scan_->range_max;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double a = (scan_->angle_min + i * scan_->angle_increment) * 180.0 / M_PI;
      if (a < from_deg || a > to_deg) {
        continue;
      }
      const double r = scan_->ranges[i];
      if (std::isfinite(r) && r >= scan_->range_min) {
        best = std::min(best, r);
      }
    }
    return best;
  }

  // ---------------------------------------------------------------------------
  // El cerebro: del scan a una velocidad. Se llama 20 veces por segundo
  // mientras está dentro de la cueva.
  // ---------------------------------------------------------------------------
  Command steer() const
  {
    Command cmd;
    // TODO: decide cmd.forward, cmd.left y cmd.yawrate con sector_min().
    //   Ideas, de la más sencilla a la mejor:
    //   1. Centrarse: compara lo que hay a la izquierda (60°..120°) con lo de
    //      la derecha (-120°..-60°) y muévete hacia el lado con más espacio,
    //      proporcional a la diferencia (y con un tope, p. ej. 0.6 m/s).
    //      Ojo: afuera de la cueva un lado puede dar range_max (30 m); usa
    //      std::min(distancia, 3.0) para que solo cuenten las paredes cerca.
    //   2. Girar con la cueva: busca el ángulo entre -75° y 75° con más
    //      espacio libre (la "salida" del túnel) y gira hacia él:
    //      yawrate = k * ángulo (en rad, con tope). Mejor aún: el espacio de
    //      cada ángulo como el MÍNIMO de una ventana de ±10° alrededor, para
    //      no meterte por una rendija donde el dron no cabe.
    //   3. Avanzar: más rápido si el frente (-15°..15°) está libre, más
    //      despacio si hay que girar mucho; nunca más de max_speed_.
    //   La seguridad de stop_distance_ se aplica después, en loop().
    //
    // Mientras tanto: derecho y despacio.
    cmd.forward = 0.3;
    return cmd;
  }

  void loop()
  {
    if (!scan_) {
      return;
    }
    const float z = static_cast<float>(-altitude_);
    const auto & l = px4_.local();

    switch (state_) {
      case State::WARMUP:
        px4_.offboard(hold_at(0.0f, 0.0f, z, 0.0f));
        // Tras ~1 s de setpoints pedir OFFBOARD y armar; y reintentar cada
        // segundo por si PX4 todavía no estaba listo (o lo rechazó).
        if (++counter_ % 20 == 0) {
          px4_.request_offboard_and_arm();
        }
        if (px4_.in_offboard()) {
          RCLCPP_INFO(get_logger(), "OFFBOARD activo, subiendo a %.1f m", altitude_);
          state_ = State::TAKEOFF;
        }
        return;

      case State::TAKEOFF:
        px4_.offboard(hold_at(0.0f, 0.0f, z, 0.0f));
        if (px4_.distance_to({0.0f, 0.0f, z}) < 0.3f) {
          RCLCPP_INFO(get_logger(), "Arriba, yendo a la boca de la cueva");
          state_ = State::TO_ENTRANCE;
        }
        break;

      case State::TO_ENTRANCE: {
          const std::array<float, 3> entrance = {static_cast<float>(entrance_n_),
            static_cast<float>(entrance_e_), z};
          px4_.offboard(hold_at(entrance[0], entrance[1], z, 0.0f));
          if (px4_.distance_to(entrance) < 0.4f) {
            RCLCPP_INFO(get_logger(), "Dentro de la boca. De aquí en adelante, solo el lidar");
            entered_ = px4_.clock();
            last_move_ = px4_.clock();
            state_ = State::CAVE;
          }
          break;
        }

      case State::CAVE: {
          if (l.x > exit_n_) {
            RCLCPP_INFO(get_logger(), "¡Afuera! Cueva cruzada en %.0f s. A la plataforma",
                        px4_.clock() - entered_);
            state_ = State::TO_PAD;
            break;
          }
          Command cmd = steer();
          // Seguridad: con algo muy cerca al frente, no avanzar (sí girar y
          // moverse de lado para encontrar por dónde).
          const double front = sector_min(-20, 20);
          if (front < stop_distance_) {
            cmd.forward = std::min(cmd.forward, 0.0);
          }
          cmd.forward = std::clamp(cmd.forward, -0.5, max_speed_);
          px4_.offboard(body_velocity(cmd, z));

          clearance_ = std::min(clearance_, sector_min(-135, 135));
          const double speed = std::hypot(l.vx, l.vy);
          if (speed > 0.15) {
            last_move_ = px4_.clock();
          } else if (px4_.clock() - last_move_ > stuck_time_) {
            RCLCPP_ERROR(get_logger(), "Atorado %.0f s sin avanzar (frente a %.1f m). Aterrizando",
                         stuck_time_, front);
            land();
            break;
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "norte %.1f m  frente %.1f  izq %.1f  der %.1f  -> adelante %.2f, izq %.2f, giro %.2f",
            l.x, front, sector_min(60, 120), sector_min(-120, -60), cmd.forward, cmd.left, cmd.yawrate);
          break;
        }

      case State::TO_PAD: {
          const std::array<float, 3> pad = {static_cast<float>(pad_n_), static_cast<float>(pad_e_), z};
          px4_.offboard(hold_at(pad[0], pad[1], z, 0.0f));
          // Aterrizar ya quieto encima: si va en movimiento, Land lo deja lejos.
          if (px4_.distance_to(pad) < 0.2f && std::hypot(l.vx, l.vy) < 0.15) {
            land();
          }
          break;
        }

      case State::LANDING:
        if (!px4_.armed()) {
          report();
          state_ = State::DONE;
        }
        return;

      case State::DONE:
        rclcpp::shutdown();
        return;
    }

    if (state_ != State::LANDING && !px4_.in_offboard()) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%u): misión abortada", px4_.status().nav_state);
      state_ = State::DONE;
    }
  }

  Setpoint hold_at(float north, float east, float z, float yaw) const
  {
    Setpoint sp;
    sp.position = {north, east, z};
    sp.yaw = yaw;
    return sp;
  }

  // Velocidad del marco del dron (adelante/izquierda) a NED, con la altura
  // fija por posición. Igual que en velocity_control: rotar por el heading.
  Setpoint body_velocity(const Command & cmd, float z) const
  {
    const double psi = px4_.local().heading;
    const double c = std::cos(psi), s = std::sin(psi);
    Setpoint sp;
    sp.position = {kNaN, kNaN, z};
    // En NED "derecha" es +y del cuerpo, así que izquierda = -y.
    sp.velocity = {static_cast<float>(cmd.forward * c + cmd.left * s),
      static_cast<float>(cmd.forward * s - cmd.left * c), kNaN};
    // En NED el yaw crece hacia la derecha: el giro a la izquierda va con signo -.
    sp.yawspeed = static_cast<float>(-cmd.yawrate);
    return sp;
  }

  void land()
  {
    px4_.land();
    RCLCPP_INFO(get_logger(), "Aterrizando");
    state_ = State::LANDING;
  }

  void report()
  {
    const auto & l = px4_.local();
    const bool crossed = l.x > exit_n_;
    const double to_pad = std::hypot(l.x - pad_n_, l.y - pad_e_);
    const bool safe = clearance_ > 0.6;
    RCLCPP_INFO(get_logger(), "Aterrizado. Resultado:");
    RCLCPP_INFO(get_logger(), "  cruzó la cueva:         %s", crossed ? "sí OK" : "no FALLÓ");
    RCLCPP_INFO(get_logger(), "  del lidar a la roca, mínimo: %.2f m (límite 0.6) %s", clearance_,
                safe ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), "  a %.2f m de la plataforma %s", to_pad, to_pad < 1.0 ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), crossed && safe && to_pad < 1.0 ? "Misión cumplida" : "Misión NO cumplida");
  }

  x500::Px4Link px4_{this};
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  LaserScan::SharedPtr scan_;

  double altitude_, max_speed_, stop_distance_, entrance_n_, entrance_e_, exit_n_, pad_n_, pad_e_;
  double stuck_time_;

  State state_ = State::WARMUP;
  int counter_ = 0;
  double entered_ = 0.0, last_move_ = 0.0;   // s, reloj de PX4
  double clearance_ = std::numeric_limits<double>::infinity();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CruzarCueva>());
  rclcpp::shutdown();
  return 0;
}
