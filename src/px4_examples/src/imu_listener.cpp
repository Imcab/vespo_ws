// =============================================================================
// imu_listener: solo LEE la IMU del dron (giroscopio + acelerómetro).
//
// Se suscribe a /fmu/out/sensor_combined, que PX4 publica a cientos de Hz con
// los datos ya calibrados y promediados de la IMU principal.
//
// Marco FRD (Forward-Right-Down), fijo al cuerpo del dron:
//   x = hacia el frente, y = hacia la derecha, z = hacia abajo.
//
// Qué esperar con el dron quieto en el suelo:
//   gyro  ≈ (0, 0, 0) rad/s
//   accel ≈ (0, 0, -9.81) m/s²  -> el acelerómetro mide la fuerza que lo
//   sostiene (el suelo empuja hacia ARRIBA, que en FRD es z negativo).
//
// Ejecutar:
//   ros2 run px4_examples imu_listener
// =============================================================================

#include <cmath>   // M_PI

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"

using px4_msgs::msg::SensorCombined;

using namespace std::chrono_literals;

class ImuListener : public rclcpp::Node
{
public:
  ImuListener()
  : Node("imu_listener")
  {
    // QoS igual al que usa PX4 al publicar.
    auto qos = rclcpp::QoS(1).best_effort().transient_local();

    // SensorCombined tiene versión 0 -> topic sin sufijo _vN.
    // El callback se ejecuta con CADA mensaje (cientos por segundo), así que
    // aquí solo guardamos el dato y contamos; imprimir aquí saturaría la terminal.
    imu_sub_ = create_subscription<SensorCombined>(
      "/fmu/out/sensor_combined", qos,
      [this](const SensorCombined::SharedPtr msg) {
        imu_ = *msg;
        msg_count_++;   // para calcular la frecuencia real de llegada
      });

    // Imprime cada 500 ms (2 Hz), suficiente para leerlo a simple vista.
    timer_ = create_wall_timer(500ms, [this]() {print_imu();});

    RCLCPP_INFO(get_logger(), "imu_listener iniciado, esperando /fmu/out/sensor_combined...");
  }

private:
  void print_imu()
  {
    // timestamp en 0 = todavía no llega ningún mensaje.
    if (imu_.timestamp == 0) {
      RCLCPP_WARN(get_logger(), "Sin datos de la IMU todavía (¿corre el agente?)");
      return;
    }

    // Frecuencia: mensajes recibidos en los últimos 0.5 s, multiplicados por 2
    // para obtener mensajes por segundo (Hz). Luego reiniciamos la cuenta.
    const double rate_hz = msg_count_ * 2.0;
    msg_count_ = 0;

    // gyro_rad y accelerometer_m_s2 son std::array<float, 3> -> [0]=x, [1]=y, [2]=z.
    // Giroscopio: velocidad angular. Lo mostramos en rad/s y en grados/s.
    RCLCPP_INFO(
      get_logger(), "Gyro  [rad/s]: x=%+.3f y=%+.3f z=%+.3f  (deg/s: %+.1f %+.1f %+.1f)",
      imu_.gyro_rad[0], imu_.gyro_rad[1], imu_.gyro_rad[2],
      rad2deg(imu_.gyro_rad[0]), rad2deg(imu_.gyro_rad[1]), rad2deg(imu_.gyro_rad[2]));

    // Acelerómetro: aceleración específica (incluye la gravedad).
    // %+.3f = siempre muestra el signo (+/-) con 3 decimales.
    RCLCPP_INFO(
      get_logger(), "Accel [m/s²]:  x=%+.3f y=%+.3f z=%+.3f  | %.0f Hz",
      imu_.accelerometer_m_s2[0], imu_.accelerometer_m_s2[1], imu_.accelerometer_m_s2[2],
      rate_hz);
  }

  // Radianes -> grados.
  static float rad2deg(float rad)
  {
    return rad * 180.0f / static_cast<float>(M_PI);
  }

  rclcpp::Subscription<SensorCombined>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  SensorCombined imu_;     // última lectura de la IMU
  int msg_count_ = 0;      // mensajes recibidos desde la última impresión
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuListener>());
  rclcpp::shutdown();
  return 0;
}
