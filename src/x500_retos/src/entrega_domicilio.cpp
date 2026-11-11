// =============================================================================
// entrega_domicilio (misión 3): lleva un medicamento de la farmacia a una casa
// a ~40 m, aterriza en el patio, espera desarmado la "entrega", vuelve a
// despegar y regresa solo a la farmacia con Return.
//
// Mundo:   vespo up --sim --world barrio_entrega
// Correr:  ros2 run x500_retos entrega_domicilio
//          ros2 run x500_retos entrega_domicilio --ros-args -p cruise_alt:=20.0 -p wait:=10.0
//
// Viene de: goto_mission (DO_REPOSITION a coordenadas GPS) + return_home
//           (Return) + set_home (DO_SET_HOME).
// Lo nuevo: una misión de varias etapas SIN offboard: el nodo solo decide qué
// sigue y PX4 vuela cada tramo con sus propios modos. Si el nodo se muere, el
// dron se queda en Hold (no hay failsafe de offboard).
//
// El patio de la entrega (la plataforma naranja) está en lat 47.3982949,
// lon 8.5463757: 36 m al norte y 16 m al este de la farmacia. La ruta pasa
// sobre la casa destino, de ~9 m de alto: cruise_alt tiene que librarla.
//
//   WAIT_GPS -> ARMING -> TAKEOFF -> FLY -> LAND_DEST -> DELIVER -> REARM
//            -> TAKEOFF_BACK -> SET_HOME -> RETURN -> DONE
//
// Criterio de éxito: la secuencia completa corre sin tocar QGroundControl,
// el log imprime cada fase y el dron termina en la H de la farmacia (< 1.5 m).
//
// TU PARTE: los TODO en DELIVER, REARM, TAKEOFF_BACK, SET_HOME y RETURN.
// La ida (hasta aterrizar en el patio) ya funciona. Las funciones de ayuda de
// abajo (take_off(), go_to_target(), set_home_to_pharmacy()...) ya están
// hechas: tu trabajo es decidir CUÁNDO llamar cada una y cuándo pasar de fase.
// =============================================================================

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "x500_retos/px4_link.hpp"

using namespace std::chrono_literals;
using x500::kNaN;
using x500::VehicleCommand;

enum class State
{
  WAIT_GPS, ARMING, TAKEOFF, FLY, LAND_DEST, DELIVER, REARM, TAKEOFF_BACK, SET_HOME, RETURN, DONE
};

class EntregaDomicilio : public rclcpp::Node
{
public:
  EntregaDomicilio()
  : Node("entrega_domicilio")
  {
    target_lat_ = declare_parameter("target_lat", 47.398294861);   // grados
    target_lon_ = declare_parameter("target_lon", 8.546375690);
    cruise_alt_ = declare_parameter("cruise_alt", 15.0);   // m sobre el suelo
    speed_ = declare_parameter("speed", 4.0);              // m/s de crucero
    wait_ = declare_parameter("wait", 5.0);                // s desarmado en el patio
    arrive_tol_ = declare_parameter("arrive_tolerance", 1.0);   // m

    timer_ = create_wall_timer(100ms, [this]() {loop();});
    RCLCPP_INFO(get_logger(), "entrega_domicilio: esperando GPS...");
  }

private:
  void loop()
  {
    switch (state_) {
      case State::WAIT_GPS:
        if (px4_.gps_ready()) {
          // La farmacia: de aquí sale y aquí tiene que regresar.
          pharmacy_lat_ = px4_.global().lat;
          pharmacy_lon_ = px4_.global().lon;
          ground_alt_ = px4_.global().alt;
          const double d = x500::gps_distance(pharmacy_lat_, pharmacy_lon_, target_lat_, target_lon_);
          RCLCPP_INFO(get_logger(), "[1/9] GPS listo. Destino a %.1f m. Armando", d);
          started_ = px4_.clock();
          state_ = State::ARMING;
        }
        break;

      case State::ARMING:
        if (arm_when_ready()) {
          take_off();
          RCLCPP_INFO(get_logger(), "[2/9] Despegando a %.0f m", cruise_alt_);
          state_ = State::TAKEOFF;
        }
        break;

      case State::TAKEOFF:
        if (at_cruise_alt()) {
          go_to_target();
          RCLCPP_INFO(get_logger(), "[3/9] A la altura, volando al patio");
          state_ = State::FLY;
        }
        break;

      case State::FLY:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "      faltan %.1f m", distance_to_target());
        if (px4_.in_hold() && distance_to_target() < arrive_tol_) {
          px4_.land();
          RCLCPP_INFO(get_logger(), "[4/9] Sobre el patio, aterrizando");
          state_ = State::LAND_DEST;
        }
        break;

      case State::LAND_DEST:
        // PX4 desarma solo unos segundos después de tocar el suelo.
        if (!px4_.armed()) {
          RCLCPP_INFO(get_logger(), "[5/9] En el patio (a %.2f m de la plataforma). Entregando...",
                      distance_to_target());
          delivered_at_ = px4_.clock();
          state_ = State::DELIVER;
        }
        break;

      case State::DELIVER:
        // TODO 1: esperar `wait_` segundos desarmado y luego pasar a REARM.
        //   delivered_at_ es el momento en que desarmó, con el reloj de PX4:
        //   el tiempo transcurrido es  px4_.clock() - delivered_at_.
        //   Imprime "[6/9] Paquete entregado...".
        RCLCPP_WARN(get_logger(), "TODO: la vuelta a casa no está hecha todavía");
        state_ = State::DONE;
        break;

      case State::REARM:
        // TODO 2: volver a armar y despegar. Mira cómo lo hace ARMING.
        break;

      case State::TAKEOFF_BACK:
        // TODO 3: esperar a estar a la altura de crucero (como TAKEOFF).
        //   OJO, la trampa: PX4 fija "casa" donde ARMA. ¿Dónde quedó casa
        //   después de volver a armar en el patio? Si mandas Return así,
        //   ¿a dónde regresa? (px4_.home() te dice dónde está casa.)
        break;

      case State::SET_HOME:
        // TODO 4: devolver casa a la farmacia (set_home_to_pharmacy()) y
        //   esperar a que PX4 la acepte antes de mandar Return:
        //   px4_.home().update_count cambia cada vez que PX4 mueve casa (ver
        //   set_home.cpp en px4_examples). Luego px4_.return_home().
        break;

      case State::RETURN:
        // TODO 5: esperar a que aterrice y desarme en la farmacia, imprimir
        //   el resumen con report() y pasar a DONE.
        break;

      case State::DONE:
        rclcpp::shutdown();   // misión terminada: ros2 run regresa a la terminal
        break;
    }
  }

  // ---------------------------------------------------------------------------
  // Ayudas (todas listas)
  // ---------------------------------------------------------------------------

  // Pasa a Hold y arma, reintentando cada segundo. true cuando ya está armado.
  bool arm_when_ready()
  {
    if (px4_.armed()) {
      return true;
    }
    if (every_second()) {
      // Desde Land (como queda tras aterrizar) PX4 no deja armar: Hold primero.
      if (!px4_.in_hold()) {
        px4_.request_hold();
      } else {
        px4_.arm();
      }
    }
    return false;
  }

  // Takeoff de PX4 sobre donde está ahora, a cruise_alt_ sobre el suelo.
  void take_off()
  {
    px4_.command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, kNaN, kNaN, kNaN, kNaN,
                 px4_.global().lat, px4_.global().lon, cruise_alt_amsl());
  }

  // DO_REPOSITION al patio: PX4 vuela solo y se queda en Hold al llegar.
  void go_to_target()
  {
    px4_.command(VehicleCommand::VEHICLE_CMD_DO_REPOSITION, static_cast<float>(speed_), 1.0f, 0.0f,
                 kNaN, target_lat_, target_lon_, cruise_alt_amsl());
  }

  // DO_SET_HOME con coordenadas (param1 = 0): casa = la farmacia, al nivel del suelo.
  void set_home_to_pharmacy()
  {
    px4_.command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0.0f, 0.0f, 0.0f, kNaN,
                 pharmacy_lat_, pharmacy_lon_, ground_alt_);
  }

  bool at_cruise_alt() const
  {
    return px4_.in_hold() && std::abs(px4_.global().alt - cruise_alt_amsl()) < 0.5;
  }

  float cruise_alt_amsl() const {return ground_alt_ + static_cast<float>(cruise_alt_);}

  double distance_to_target() const
  {
    return x500::gps_distance(px4_.global().lat, px4_.global().lon, target_lat_, target_lon_);
  }

  double distance_to_pharmacy() const
  {
    return x500::gps_distance(px4_.global().lat, px4_.global().lon, pharmacy_lat_, pharmacy_lon_);
  }

  // Qué tan lejos de la farmacia está "casa" según PX4.
  double home_offset() const
  {
    return x500::gps_distance(px4_.home().lat, px4_.home().lon, pharmacy_lat_, pharmacy_lon_);
  }

  void report()
  {
    const double error = distance_to_pharmacy();
    const bool ok = error < 1.5;
    RCLCPP_INFO(get_logger(), "De vuelta en la farmacia y desarmado. Misión de %.0f s",
                px4_.clock() - started_);
    RCLCPP_INFO(get_logger(), "  a %.2f m de la H %s", error, ok ? "OK" : "FALLÓ (¿a dónde apuntaba casa?)");
    RCLCPP_INFO(get_logger(), ok ? "Misión cumplida" : "Misión NO cumplida");
  }

  bool every_second()
  {
    if ((now() - last_command_).seconds() < 1.0) {
      return false;
    }
    last_command_ = now();
    return true;
  }

  x500::Px4Link px4_{this};
  rclcpp::TimerBase::SharedPtr timer_;

  double target_lat_, target_lon_, cruise_alt_, speed_, wait_, arrive_tol_;

  State state_ = State::WAIT_GPS;
  double pharmacy_lat_ = 0.0, pharmacy_lon_ = 0.0;
  float ground_alt_ = 0.0f;
  uint32_t home_count_ = 0;
  double started_ = 0.0, delivered_at_ = 0.0;   // s, reloj de PX4
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EntregaDomicilio>());
  rclcpp::shutdown();
  return 0;
}
