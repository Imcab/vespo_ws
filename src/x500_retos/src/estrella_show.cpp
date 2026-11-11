// =============================================================================
// estrella_show (misión 1): dibuja una estrella de N picos en el cielo para el
// show de año nuevo, pasando por los aros de las balizas de la plaza.
//
// Mundo:   vespo up --sim --world plaza_show
// Correr:  ros2 run x500_retos estrella_show
//          ros2 run x500_retos estrella_show --ros-args -p points:=6 -p radius:=4.0
//          ros2 run x500_retos estrella_show --ros-args -p points:=4 -p inner_ratio:=0.0   # cuadrado
//
// Viene de: square_mission (waypoints en offboard) + orbit_mission (parámetros).
// Lo nuevo: los waypoints ya no se escriben a mano, se GENERAN con una fórmula.
//
// La estrella en NED, vista desde arriba (x = norte arriba, y = este a la derecha):
//
//                   P0                P = pico, en el círculo de radio R,
//                  /  \                   repartidos cada 360°/N empezando al norte.
//     P4 _______ I4    I0 _______ P1  I = vértice interior, en el círculo de radio
//        `-._              _.-´           R * inner_ratio, a medio camino en ángulo
//            I3          I1               entre dos picos.
//           /     I2      \  (centro)
//         P3 ___.´  `.___ P2          inner_ratio = 0.382 -> estrella regular de 5
//                                     inner_ratio = 0     -> polígono (solo picos)
//
//   Orden de vuelo: P0, I0, P1, I1, ... y de vuelta a P0.
//
// Criterio de éxito: pasa a menos de `tolerance` (0.5 m) de cada vértice,
// regresa al centro y aterriza. Al final imprime qué tan cerca pasó de cada uno.
//
// TU PARTE: los TODO 1 y 2 en vertices(). Todo lo demás ya funciona: si
// vertices() regresa una lista vacía, el dron despega, avisa y aterriza.
// =============================================================================

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using x500::Setpoint;
using Point = std::array<float, 3>;   // {norte, este, abajo} en metros

//   WARMUP -> TAKEOFF -> STAR -> CENTER -> LANDING -> DONE
enum class State { WARMUP, TAKEOFF, STAR, CENTER, LANDING, DONE };

class EstrellaShow : public rclcpp::Node
{
public:
  EstrellaShow()
  : Node("estrella_show")
  {
    points_ = declare_parameter("points", 5);              // picos de la estrella
    radius_ = declare_parameter("radius", 5.0);            // m, del centro a un pico
    inner_ratio_ = declare_parameter("inner_ratio", 0.382);  // 0 = polígono
    altitude_ = declare_parameter("altitude", 6.0);        // m sobre el despegue
    tolerance_ = declare_parameter("tolerance", 0.5);      // m para dar un vértice por visitado
    hold_ = declare_parameter("hold", 0.5);                // s quieto en cada vértice

    // 20 Hz: más que suficiente para el heartbeat de offboard (pide > 2 Hz).
    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "estrella_show: %ld picos, radio %.1f m, a %.1f m de altura",
                points_, radius_, altitude_);
  }

private:
  // ---------------------------------------------------------------------------
  // Los vértices de la estrella (o polígono) en NED, en el orden en que se
  // vuelan: pico 0, interior 0, pico 1, interior 1, ... Todos a -altitude_.
  // ---------------------------------------------------------------------------
  std::vector<Point> vertices() const
  {
    std::vector<Point> out;
    const float z = static_cast<float>(-altitude_);
    // TODO 1: los picos. Hay points_ picos repartidos en 360°, el primero al
    //   norte. Un punto a distancia r y ángulo a medido desde el norte hacia
    //   el este está en  {r * cos(a), r * sin(a), z}  (NED: x norte, y este).
    //   Pista: el ángulo del pico k es 2*pi*k / points_ (en radianes).
    //
    // TODO 2: los vértices interiores. Si inner_ratio_ > 0, después de cada
    //   pico va un vértice a radio radius_ * inner_ratio_, con un ángulo a la
    //   mitad entre ese pico y el siguiente. Si inner_ratio_ == 0 no hay
    //   interiores y queda un polígono: con points:=4 debe salir un cuadrado.
    //
    // Para agregar un punto:  out.push_back({norte, este, z});
    // Ojo con los tipos: radius_ es double y Point lleva float
    // (static_cast<float>(...)).
    (void)z;
    return out;
  }

  void loop()
  {
    const Point center = {0.0f, 0.0f, static_cast<float>(-altitude_)};

    switch (state_) {
      case State::WARMUP:
        // ~1 s de setpoints antes de pedir OFFBOARD (si no, PX4 lo rechaza).
        px4_.offboard(hover(center));
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
        px4_.offboard(hover(center));
        if (px4_.distance_to(center) < tolerance_) {
          path_ = vertices();
          if (path_.empty()) {
            RCLCPP_ERROR(get_logger(), "vertices() regresó una lista vacía: falta el TODO. Aterrizando");
            land();
            return;
          }
          closest_.assign(path_.size(), 1e9f);
          RCLCPP_INFO(get_logger(), "En el centro. La figura tiene %zu vértices", path_.size());
          index_ = 0;
          state_ = State::STAR;
        }
        break;

      case State::STAR: {
          const Point & target = path_[index_];
          px4_.offboard(hover(target));
          const float d = px4_.distance_to(target);
          closest_[index_] = std::min(closest_[index_], d);
          if (d < tolerance_) {
            // Quieto `hold_` segundos en el vértice, para que se note la esquina.
            if (arrived_ < 0.0) {
              arrived_ = px4_.clock();
            } else if (px4_.clock() - arrived_ >= hold_) {
              RCLCPP_INFO(get_logger(), "Vértice %zu/%zu (%.2f m)", index_ + 1, path_.size(),
                          closest_[index_]);
              arrived_ = -1.0;
              if (++index_ == path_.size()) {
                RCLCPP_INFO(get_logger(), "Figura completa, regresando al centro");
                state_ = State::CENTER;
              }
            }
          }
          break;
        }

      case State::CENTER:
        px4_.offboard(hover(center));
        if (px4_.distance_to(center) < tolerance_) {
          land();
        }
        break;

      case State::LANDING:
        if (!px4_.armed()) {
          report();
          state_ = State::DONE;
        }
        return;

      case State::DONE:
        rclcpp::shutdown();   // misión terminada: ros2 run regresa a la terminal
        return;
    }

    if (state_ != State::LANDING && !px4_.in_offboard()) {
      RCLCPP_WARN(get_logger(), "Fuera de OFFBOARD (nav=%u): misión abortada", px4_.status().nav_state);
      state_ = State::DONE;
      return;
    }
    if (state_ == State::STAR) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "alt=%.1f m  rumbo al vértice %zu/%zu",
                           -px4_.local().z, index_ + 1, path_.size());
    }
  }

  // Quedarse en un punto mirando al norte (yaw 0: la nariz hacia el escenario).
  Setpoint hover(const Point & p) const
  {
    Setpoint sp;
    sp.position = p;
    sp.yaw = 0.0f;
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
    RCLCPP_INFO(get_logger(), "Aterrizado. Qué tan cerca pasó de cada vértice:");
    bool ok = !closest_.empty();
    for (size_t i = 0; i < closest_.size(); ++i) {
      const bool hit = closest_[i] < tolerance_;
      ok = ok && hit;
      RCLCPP_INFO(get_logger(), "  %2zu: %.2f m %s", i + 1, closest_[i], hit ? "OK" : "FALLÓ");
    }
    RCLCPP_INFO(get_logger(), ok ? "Misión cumplida" : "Misión NO cumplida");
  }

  x500::Px4Link px4_{this};
  rclcpp::TimerBase::SharedPtr timer_;

  int64_t points_;
  double radius_, inner_ratio_, altitude_, tolerance_, hold_;

  State state_ = State::WARMUP;
  int counter_ = 0;
  std::vector<Point> path_;
  std::vector<float> closest_;   // la distancia mínima a la que pasó de cada vértice
  size_t index_ = 0;
  double arrived_ = -1.0;   // s (reloj de PX4) en que llegó al vértice; -1 = todavía no
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstrellaShow>());
  rclcpp::shutdown();
  return 0;
}
