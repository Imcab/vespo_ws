// =============================================================================
// takeoff_mission: despega el dron en modo OFFBOARD y lo mantiene a 5 m.
//
// Es lo que construimos en el paso 2. Flujo:
//   1. Durante ~1 s publica heartbeat (OffboardControlMode) + setpoint
//      (TrajectorySetpoint), porque PX4 rechaza entrar a offboard si no le
//      están llegando setpoints ANTES de pedirlo.
//   2. En el ciclo 10 (1 s) manda los comandos OFFBOARD y ARM, una sola vez.
//   3. Sigue publicando heartbeat + setpoint para siempre. Si se dejan de
//      publicar (p. ej. Ctrl+C), PX4 activa el failsafe (COM_OBL_RC_ACT).
//
// Ejecutar (con PX4 SITL y el agente corriendo):
//   ros2 run px4_examples takeoff_mission
// =============================================================================

// rclcpp = la librería de ROS 2 para C++ (el equivalente de "import rclpy").
#include "rclcpp/rclcpp.hpp"

// Mensajes de PX4. El nombre del archivo va en snake_case aunque el tipo sea
// CamelCase: VehicleStatus -> vehicle_status.hpp.
#include "px4_msgs/msg/vehicle_status.hpp"           // estado: armado, modo de vuelo...
#include "px4_msgs/msg/vehicle_local_position.hpp"   // posición x, y, z en NED
#include "px4_msgs/msg/offboard_control_mode.hpp"    // heartbeat: qué tipo de control usamos
#include "px4_msgs/msg/trajectory_setpoint.hpp"      // a dónde queremos ir
#include "px4_msgs/msg/vehicle_command.hpp"          // comandos: armar, cambiar de modo, aterrizar

// "using" nos deja escribir VehicleStatus en vez de px4_msgs::msg::VehicleStatus.
// Es como "from px4_msgs.msg import VehicleStatus" en Python.
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

// Permite escribir tiempos como 100ms o 1s directamente en el código.
using namespace std::chrono_literals;

// Nuestra clase hereda de rclcpp::Node (en Python: class TakeoffMission(Node)).
class TakeoffMission : public rclcpp::Node
{
public:
  // Constructor: se ejecuta una vez al crear el nodo (el __init__ de Python).
  // ": Node("takeoff_mission")" llama al constructor del padre con el nombre
  // del nodo (en Python: super().__init__('takeoff_mission')).
  TakeoffMission()
  : Node("takeoff_mission")
  {
    // QoS (calidad de servicio). PX4 publica con best_effort + transient_local
    // (verificado en PX4-Autopilot/src/modules/uxrce_dds_client/utilities.hpp).
    // Con el QoS por defecto (reliable) la suscripción NO recibe nada y no da
    // ningún error. El "1" es la profundidad de la cola: solo nos importa el
    // último mensaje.
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    // ---------------------------------------------------------------------
    // Suscripciones (lo que PX4 nos manda: topics /fmu/out/...)
    // ---------------------------------------------------------------------

    // create_subscription<Tipo>(topic, qos, callback).
    // El callback es una "lambda": una función sin nombre escrita en línea.
    //   [this]        -> la lambda puede usar los atributos del nodo (status_)
    //   (... msg)     -> recibe el mensaje como SharedPtr (un puntero)
    //   status_ = *msg -> el * "desreferencia" el puntero: copia el mensaje
    // El topic lleva _v4 porque VehicleStatus tiene MESSAGE_VERSION = 4.
    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", qos,
      [this](const VehicleStatus::SharedPtr msg) {
        status_ = *msg;
      });

    // Igual que la anterior. _v1 porque VehicleLocalPosition tiene versión 1.
    position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {
        position_ = *msg;
      });

    // ---------------------------------------------------------------------
    // Publishers (lo que nosotros le mandamos a PX4: topics /fmu/in/...)
    // Estos tres mensajes tienen versión 0, por eso no llevan sufijo _vN.
    // ---------------------------------------------------------------------
    offboard_pub_ = create_publisher<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos);
    command_pub_ = create_publisher<VehicleCommand>(
      "/fmu/in/vehicle_command", qos);

    // Timer que llama a loop() cada 100 ms (10 Hz). PX4 exige recibir el
    // heartbeat a más de 2 Hz; 10 Hz nos da margen.
    // La lambda [this]() { loop(); } no recibe nada: solo llama al método.
    timer_ = create_wall_timer(100ms, [this]() {loop();});

    RCLCPP_INFO(get_logger(), "takeoff_mission iniciado, calentando setpoints...");
  }

private:
  // Se ejecuta cada 100 ms.
  void loop()
  {
    // --- 1) Heartbeat ------------------------------------------------------
    // Le dice a PX4 "sigo vivo y voy a controlar por POSICIÓN".
    // Las {} inicializan todos los campos en 0 / false; solo activamos position.
    OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.timestamp = now_us();
    offboard_pub_->publish(offboard_msg);   // -> porque offboard_pub_ es un puntero

    // --- 2) Setpoint -------------------------------------------------------
    // A dónde queremos que vaya el dron. Marco NED (North-East-Down):
    //   x = norte, y = este, z = abajo  -> z = -5 significa 5 m ARRIBA.
    // position es un std::array<float, 3>; la "f" indica literal float.
    TrajectorySetpoint setpoint_msg{};
    setpoint_msg.position = {0.0f, 0.0f, -5.0f};
    setpoint_msg.yaw = 0.0f;                // rad; 0 = mirando al norte
    setpoint_msg.timestamp = now_us();
    setpoint_pub_->publish(setpoint_msg);

    // --- 3) Tras ~1 s de setpoints: pedir OFFBOARD y ARMAR (una sola vez) ---
    // OJO: "==" compara; "=" asignaría y el if sería siempre verdadero.
    if (counter_ == 10) {
      // DO_SET_MODE: param1 = 1 (modo "custom" de PX4), param2 = 6 (OFFBOARD).
      send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      // ARM_DISARM: param1 = 1 armar, 0 desarmar.
      send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      RCLCPP_INFO(get_logger(), "Comandos OFFBOARD + ARM enviados");
    }

    // El contador se detiene en 11 para que el "== 10" solo se cumpla una vez.
    if (counter_ <= 10) {
      counter_++;
    }

    // --- 4) Log: máximo una vez por segundo (1000 ms) ----------------------
    // Formato estilo printf: %d = entero, %.2f = float con 2 decimales.
    // arming: 1 = desarmado, 2 = armado.  nav: 4 = Hold, 14 = OFFBOARD.
    // La altura es -z porque NED apunta hacia abajo.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "arming=%d nav=%d alt=%.2f m",
      status_.arming_state, status_.nav_state, -position_.z);
  }

  // Tiempo actual en microsegundos, que es la unidad de los timestamps de PX4.
  uint64_t now_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  // Arma y publica un VehicleCommand (equivalente a un comando MAVLink).
  // "= 0.0f" son parámetros por defecto, igual que p1=0.0 en Python.
  void send_command(uint32_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    VehicleCommand msg{};           // variable local: se crea, se llena y se publica
    msg.command = command;          // qué comando (constantes VEHICLE_CMD_*)
    msg.param1 = p1;                // el significado de cada param depende del comando
    msg.param2 = p2;
    msg.target_system = 1;          // a quién va dirigido: el sistema 1 (el dron)
    msg.target_component = 1;       // componente 1 = el autopiloto
    msg.source_system = 1;          // quién lo manda
    msg.source_component = 1;
    msg.from_external = true;       // viene de fuera de PX4 (nosotros, por ROS 2)
    msg.timestamp = now_us();
    command_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // Atributos. En C++ hay que declarar su tipo (en Python bastaba self.x = ...).
  // Suscripciones, publishers y timer DEBEN guardarse aquí: si fueran variables
  // locales del constructor se destruirían al salir de él y dejarían de funcionar.
  // El "_" al final es la convención para atributos privados.
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Últimos mensajes recibidos (los guardan los callbacks y los lee loop()).
  // Los mensajes que PUBLICAMOS no van aquí: son variables locales en loop().
  VehicleStatus status_;
  VehicleLocalPosition position_;

  // Cuántos ciclos de loop() han pasado (cada ciclo = 100 ms).
  int counter_ = 0;
};

int main(int argc, char ** argv)
{
  // Inicializa ROS 2 e instala el manejador de Ctrl+C.
  rclcpp::init(argc, argv);
  // Crea el nodo y lo mantiene procesando callbacks y timers.
  // Con Ctrl+C, spin() retorna normalmente (no hay excepción como en Python)
  // y el nodo se destruye solo al no tener más dueños (RAII).
  rclcpp::spin(std::make_shared<TakeoffMission>());
  // Apaga ROS 2 limpiamente.
  rclcpp::shutdown();
  return 0;
}
