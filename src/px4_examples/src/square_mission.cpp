#include <array>    // std::array  -> un punto de 3 floats {x, y, z}
#include <cmath>    // std::sqrt   -> raíz cuadrada
#include <vector>   // std::vector -> la lista de waypoints (como una list de Python)

#include "rclcpp/rclcpp.hpp" // client wrapper de ros
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;

// Para usar el tiempo, ej 100ms, el decorador "ms"
using namespace std::chrono_literals;

// Estados de la misión (en Python: class State(Enum)). Se usan como State::WARMUP.
//
//   WARMUP  -> MISSION -> LANDING -> DONE
//   (armar y    (recorrer    (PX4        (fin)
//    offboard)   waypoints)   aterriza)
enum class State { WARMUP, MISSION, LANDING, DONE };

// La clase, heredando de nodo
class Controller : public rclcpp::Node{

  public:

    // mi constructor, tambien inicializa el constructor de la case padre, como un super o un self.init por ej
    Controller(): Node("square_mission"){ // "square mission es el nombre del nodo"

      // creamos un QOS (quality of service), que va a "configurar" como se manda la informacion con el dds
      // en vez de "fijar" un tamaño de la cola predeterminado
      // usamos auto para no escribir mucho, como si fuera un "var" en Java
      auto qos = rclcpp::QoS(1).best_effort().transient_local();

      // Crea un subscriptor de un VehicleStatus que es practicamente todo el status del vehiculo, lo usamos como
      // heartbeat
      status_sub_ = create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status_v4", qos,
        [this](const VehicleStatus::SharedPtr msg){
          status_ = *msg;
      });


      position_sub_ = create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", qos,
        [this](const VehicleLocalPosition::SharedPtr msg){
          position_ = *msg;
        }
      );

      offboard_pub_ = create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);

      traj_pub_ = create_publisher<TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos
      );

      veh_pub_ = create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", qos
      );

      timer_ = create_wall_timer(100ms, [this](){loop();});

    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
    VehicleStatus status_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
    VehicleLocalPosition position_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;

    rclcpp::Publisher<VehicleCommand>::SharedPtr veh_pub_;

    int counter_ = 0;

    // ---------------------------------------------------------------------
    // Misión
    // ---------------------------------------------------------------------

    State state_ = State::WARMUP;   // en qué parte de la misión vamos
    size_t wp_index_ = 0;           // qué waypoint toca (size_t = entero sin signo, para índices)

    // Waypoints en NED: x = norte, y = este, z = abajo (z = -5 -> 5 m ARRIBA).
    //
    //    x (Norte)
    //    ▲
    //  5 │  WP1 ●────────● WP2
    //    │      │        │
    //  0 │  WP0 ●────────● WP3
    //    │     WP4 (= WP0)
    //    └──────────────────► y (Este)
    //       0            5
    std::vector<std::array<float, 3>> waypoints_ = {
      {0.0f, 0.0f, -5.0f},   // WP0: despegue
      // TODO 1: agrega WP1, WP2, WP3 y WP4 siguiendo el dibujo de arriba
      {5.0f, 0.0f, -5.0f},
      {5.0f, 5.0f, -5.0f},
      {0.0f,5.0f, -5.0f},
      {0.0f,0.0f,-5.0f}

    };

    void loop(){

      // ===== 1) Heartbeat + setpoint: SOLO mientras nosotros controlamos =====
      // En LANDING y DONE ya no: PX4 está en AUTO_LAND y no usa nuestros setpoints.
      if (state_ == State::WARMUP || state_ == State::MISSION) {

        OffboardControlMode offmsg{};
        offmsg.position = true;
        offmsg.timestamp = now_us();

        offboard_pub_->publish(offmsg);

        TrajectorySetpoint trajmsg{};
        trajmsg.position = waypoints_[wp_index_];   // el waypoint que toca (antes era {0, 0, -5} fijo)
        trajmsg.yaw = 0.0f;
        trajmsg.timestamp = now_us();

        traj_pub_->publish(trajmsg);
      }

      // ===== 2) Qué hacer según el estado =====
      // OJO: cada case termina en "break;". Si lo olvidas, sigue ejecutando el case de abajo.
      switch (state_) {

        case State::WARMUP:
          // Lo del paso 2: tras ~1 s de setpoints, pedir OFFBOARD y ARMAR una sola vez
          if(counter_ == 10){
            send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
            send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
          }

          if (counter_ <= 10) {
            counter_++;
          }

          if(status_.arming_state == VehicleStatus::ARMING_STATE_ARMED && status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD){
            RCLCPP_INFO(get_logger(), "Offboard activo, iniciando mision");
            state_ = State::MISSION;
          }
          break;

        case State::MISSION:

          if(reached(waypoints_[wp_index_])){
            RCLCPP_INFO(get_logger(), "Llegue al WP %zu", wp_index_);
            wp_index_++;

            if(wp_index_ == waypoints_.size()){
              send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
              RCLCPP_INFO(get_logger(), "Mision completa, aterrizando");
              state_ = State::LANDING;
            }
          }

          break;

        case State::LANDING:
          // PX4 desarma solo al tocar tierra. Cuando pase, terminamos.

          if(status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED){
            state_ = State::DONE;
          }

          break;

        case State::DONE:
          RCLCPP_INFO(get_logger(), "Aterrizado");

          // nada que hacer
          break;
      }

      // ===== 3) Log, máximo una vez por segundo =====
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "arming=%d nav=%d alt=%.2f m wp=%zu",
      status_.arming_state, status_.nav_state, -position_.z, wp_index_
      );

    }

    bool reached(const std::array<float, 3> & target){

      float dx = position_.x - target[0];
      float dy = position_.y - target[1];
      float dz = position_.z - target[2];

      float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

      return distance <0.3f;
    }

    uint64_t now_us(){
      return get_clock()->now().nanoseconds() / 1000;
    }

    void send_command(uint32_t command, float p1 = 0.0f , float p2 = 0.0f){
      VehicleCommand msg{};

      msg.command = command;
      msg.param1 = p1;
      msg.param2 = p2;
      msg.target_system = 1;
      msg.target_component = 1;
      msg.source_system = 1;
      msg.source_component = 1;
      msg.from_external = true;
      msg.timestamp = now_us();
      veh_pub_ -> publish(msg);

    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
