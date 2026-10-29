// =============================================================================
// gps_status: muestra la calidad del GPS y dice si ya se puede volar con él.
//
// Lee /fmu/out/vehicle_gps_position (SensorGps, versión 0: sin sufijo _vN).
// Es el dato CRUDO del receptor. La posición que usa PX4 para volar es la del
// estimador (vehicle_global_position, ver telemetry_listener), que mezcla el
// GPS con la IMU, el barómetro, etc.
//
//   fix_type         0-1 sin fix, 2 = 2D, 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed
//   satellites_used  satélites usados en la solución
//   eph / epv        error estimado horizontal / vertical (m)
//   hdop             qué tan buena es la geometría de los satélites (< 2 es buena)
//   jamming_state    0 desconocido, 1 OK, 2 mitigado, 3 interferencia detectada
//
// Ejecutar (solo lee):
//   ros2 run px4_examples gps_status
//   ros2 run px4_examples gps_status --ros-args -p min_satellites:=8 -p max_eph:=2.0
// =============================================================================

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/sensor_gps.hpp"

using px4_msgs::msg::SensorGps;

using namespace std::chrono_literals;

class GpsStatus : public rclcpp::Node
{
public:
  GpsStatus()
  : Node("gps_status")
  {
    // Umbrales para decir "listo". Los de PX4 para armar están en
    // COM_ARM_EKF_POS / EKF2_REQ_NSATS / EKF2_REQ_EPH.
    min_satellites_ = declare_parameter("min_satellites", 6);
    max_eph_ = declare_parameter("max_eph", 3.0);

    auto qos = rclcpp::QoS(1).best_effort().transient_local();
    gps_sub_ = create_subscription<SensorGps>(
      "/fmu/out/vehicle_gps_position", qos,
      [this](const SensorGps::SharedPtr msg) {gps_ = *msg; received_ = true;});

    // El GPS llega a ~5-10 Hz; imprimimos un resumen cada segundo.
    timer_ = create_wall_timer(1s, [this]() {report();});
  }

private:
  void report()
  {
    if (!received_) {
      RCLCPP_WARN(get_logger(), "Sin datos de GPS. ¿Corre el agente? ¿Hay GPS conectado?");
      return;
    }

    // Revisamos cada requisito por separado para decir QUÉ falta.
    std::string missing;
    if (gps_.fix_type < SensorGps::FIX_TYPE_3D) {
      missing += " sin fix 3D;";
    }
    if (gps_.satellites_used < min_satellites_) {
      missing += " pocos satélites;";
    }
    if (gps_.eph > max_eph_) {
      missing += " error horizontal alto;";
    }
    if (gps_.jamming_state == 3 || gps_.spoofing_state == 3) {
      missing += " interferencia o spoofing;";
    }

    RCLCPP_INFO(get_logger(), "%s | %d sat | eph %.1f m epv %.1f m | hdop %.1f | %.7f, %.7f",
                fix_name(gps_.fix_type), gps_.satellites_used, gps_.eph, gps_.epv, gps_.hdop,
                gps_.latitude_deg, gps_.longitude_deg);
    if (missing.empty()) {
      RCLCPP_INFO(get_logger(), "  GPS listo para volar");
    } else {
      RCLCPP_WARN(get_logger(), "  Todavía no:%s", missing.c_str());
    }
  }

  static const char * fix_name(uint8_t fix)
  {
    switch (fix) {
      case SensorGps::FIX_TYPE_2D: return "Fix 2D";
      case SensorGps::FIX_TYPE_3D: return "Fix 3D";
      case SensorGps::FIX_TYPE_RTCM_CODE_DIFFERENTIAL: return "DGPS";
      case SensorGps::FIX_TYPE_RTK_FLOAT: return "RTK float";
      case SensorGps::FIX_TYPE_RTK_FIXED: return "RTK fixed";
      case SensorGps::FIX_TYPE_EXTRAPOLATED: return "Extrapolado";
      default: return "Sin fix";
    }
  }

  rclcpp::Subscription<SensorGps>::SharedPtr gps_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  SensorGps gps_;
  bool received_ = false;
  int64_t min_satellites_;
  double max_eph_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsStatus>());
  rclcpp::shutdown();
  return 0;
}
