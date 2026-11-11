// =============================================================================
// guardian_geocerca (misión 4): un nodo que NO vuela, solo vigila. En el
// concierto el dron no puede salirse de una caja de 20 x 20 x 8 m. Si va a
// salirse, el guardián le quita el control (Hold) y, ya quieto, lo manda a
// casa (Return). No importa quién lo esté volando: un nodo, QGC o un piloto.
//
// Mundo:   vespo up --sim --world plaza_concierto
// Correr:  ros2 run x500_retos guardian_geocerca                   # terminal 2
//          ros2 run x500_retos vuelo_prueba                        # terminal 3: el que se escapa
//          ros2 run x500_retos vuelo_prueba --ros-args -p direction_deg:=90.0 -p speed:=6.0
//   (o vuela a mano desde QGroundControl e intenta salirte de la caja)
//
// Viene de: flight_mode_monitor + telemetry_listener (solo leer) y
//           return_home (comandos de modo).
//
// La caja en NED desde el despegue (la caja roja del mundo):
//   norte  -6 .. 14 m      este  -10 .. 10 m      altura  hasta 8 m
//
// ¿Por qué no basta con revisar si ya se salió? Porque el dron no frena en
// seco: a 4 m/s el x500 de SITL en Hold recorre todavía ~4.5 m (frena a
// ~2.4 m/s² y tarda en reaccionar). Hay que
// PREDECIR dónde se va a detener y actuar antes:
//
//   distancia de frenado = v * t_reacción  +  v² / (2 a)
//
// Criterio de éxito: el dron se detiene antes de pasar 1 m la cerca (la línea
// naranja del piso), sin importar dirección ni velocidad (prueba 2 y 6 m/s).
//
// OJO con Return: sube a RTL_RETURN_ALT antes de volver (30 m en el x500 de
// SITL), o sea que se sale por el techo. En un concierto real eso se
// configura en QGC (Parámetros > Return Mode) por debajo de la cerca.
//
// TU PARTE: TODO 1 (stop_point), TODO 2 (cuándo mandar Hold, en WATCH) y
// TODO 3 (cuándo mandar Return, en BRAKING). La medición y el reporte ya
// funcionan: si no haces nada, el guardián solo mira y al final dice que falló.
// =============================================================================

#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using Vec3 = std::array<double, 3>;   // NED: norte, este, abajo

//   WATCH -> BRAKING -> RETURNING -> (aterriza) -> WATCH
enum class State { WATCH, BRAKING, RETURNING };

class GuardianGeocerca : public rclcpp::Node
{
public:
  GuardianGeocerca()
  : Node("guardian_geocerca")
  {
    north_min_ = declare_parameter("north_min", -6.0);
    north_max_ = declare_parameter("north_max", 14.0);
    east_min_ = declare_parameter("east_min", -10.0);
    east_max_ = declare_parameter("east_max", 10.0);
    alt_max_ = declare_parameter("alt_max", 8.0);
    tolerance_ = declare_parameter("tolerance", 1.0);     // m que se permite rebasar
    brake_accel_ = declare_parameter("brake_accel", 2.0);  // m/s² con que frena Hold (medido: ~2.4)
    reaction_ = declare_parameter("reaction", 0.5);        // s entre detectar y que empiece a frenar

    timer_ = create_wall_timer(50ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "Guardián listo: norte %.0f..%.0f, este %.0f..%.0f, hasta %.0f m",
                north_min_, north_max_, east_min_, east_max_, alt_max_);
  }

private:
  // ---------------------------------------------------------------------------
  // Cuánto está AFUERA de la caja un punto NED, en metros (0 si está adentro).
  // ---------------------------------------------------------------------------
  double outside_by(const Vec3 & p) const
  {
    const double dn = std::max({north_min_ - p[0], 0.0, p[0] - north_max_});
    const double de = std::max({east_min_ - p[1], 0.0, p[1] - east_max_});
    const double du = std::max(-p[2] - alt_max_, 0.0);   // altura = -z (NED)
    return std::sqrt(dn * dn + de * de + du * du);
  }

  // ---------------------------------------------------------------------------
  // Dónde se detendría el dron si en este instante mandamos Hold.
  // ---------------------------------------------------------------------------
  Vec3 stop_point() const
  {
    const auto & l = px4_.local();
    Vec3 p = {l.x, l.y, l.z};
    // TODO 1: predecir el punto donde se detiene. Con la velocidad actual
    //   (l.vx, l.vy, l.vz en m/s, NED), cada eje avanza:
    //     v * reaction_          (lo que tarda en reaccionar)
    //   + v * |v| / (2 a)        (frenando con a = brake_accel_; el v * |v|
    //                             conserva el signo: hacia dónde va)
    //   Regresa p + ese avance.
    //   Mientras tanto regresa la posición actual (sin predecir nada).
    return p;
  }

  void loop()
  {
    const auto & l = px4_.local();
    const Vec3 here = {l.x, l.y, l.z};

    // Fuera de vuelo no hay nada que vigilar; al aterrizar, reporte.
    if (!px4_.armed()) {
      if (flying_) {
        report();
        flying_ = false;
      }
      state_ = State::WATCH;
      return;
    }
    flying_ = true;
    // Lo que de verdad se salió mientras el guardián era responsable (no
    // cuenta la subida de Return, que es configuración de PX4).
    if (state_ != State::RETURNING) {
      worst_ = std::max(worst_, outside_by(here));
    }

    switch (state_) {
      case State::WATCH:
        // TODO 2: si el punto donde se detendría (stop_point()) queda afuera
        //   de la caja (outside_by(...) > 0), mandar Hold con
        //   px4_.request_hold(), avisar con un RCLCPP_WARN y pasar a BRAKING.
        //   Pista: guarda el momento (px4_.clock()) para el TODO 3.
        break;

      case State::BRAKING:
        // TODO 3: esperar a que esté casi quieto (speed() < 0.3 m/s) y en Hold
        //   (px4_.in_hold()); entonces px4_.return_home() y pasar a
        //   RETURNING. Si en 1 s no entró a Hold, volver a pedirlo (alguien
        //   puede estar peleando el control).
        break;

      case State::RETURNING:
        if (-l.z > alt_max_ + tolerance_ && !warned_rtl_) {
          RCLCPP_WARN(get_logger(), "Return subió a %.0f m, arriba de la cerca: baja RTL_RETURN_ALT en QGC",
                      -l.z);
          warned_rtl_ = true;
        }
        break;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "vigilando: a %.1f m de la orilla, %.1f m/s",
                         distance_to_edge(here), speed());
  }

  double speed() const
  {
    const auto & l = px4_.local();
    return std::sqrt(l.vx * l.vx + l.vy * l.vy + l.vz * l.vz);
  }

  // Distancia a la pared más cercana (negativa si ya está afuera).
  double distance_to_edge(const Vec3 & p) const
  {
    if (outside_by(p) > 0.0) {
      return -outside_by(p);
    }
    return std::min({p[0] - north_min_, north_max_ - p[0], p[1] - east_min_, east_max_ - p[1],
        alt_max_ + p[2]});
  }

  void report()
  {
    const bool ok = worst_ <= tolerance_;
    RCLCPP_INFO(get_logger(), "Aterrizó. Lo más que se salió de la cerca: %.2f m (límite %.1f) %s",
                worst_, tolerance_, ok ? "OK" : "FALLÓ");
    RCLCPP_INFO(get_logger(), ok ? "Misión cumplida" : "Misión NO cumplida");
    worst_ = 0.0;
    warned_rtl_ = false;
  }

  x500::Px4Link px4_{this};
  rclcpp::TimerBase::SharedPtr timer_;

  double north_min_, north_max_, east_min_, east_max_, alt_max_;
  double tolerance_, brake_accel_, reaction_;

  State state_ = State::WATCH;
  bool flying_ = false, warned_rtl_ = false;
  double worst_ = 0.0;            // m, lo más que se salió en este vuelo
  double braking_since_ = 0.0;    // s, reloj de PX4
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuardianGeocerca>());
  rclcpp::shutdown();
  return 0;
}
