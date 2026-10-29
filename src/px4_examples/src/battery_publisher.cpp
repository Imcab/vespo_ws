// =============================================================================
// battery_publisher: lee la batería de PX4 y publica SOLO el porcentaje en un
// topic NUESTRO, /vespo/battery_percent (std_msgs/Float32).
//
// Es el patrón de "mandar un valor" entre nodos de ROS 2: un nodo traduce lo
// que dice PX4 a algo simple, y cualquier otro nodo (battery_subscriber, una
// interfaz, la misión) lo usa sin saber nada de px4_msgs.
//
// Diferencias con los topics de PX4:
//   - El nombre lo elegimos nosotros. Convención del equipo: /vespo/...
//   - QoS por defecto (reliable): entre nodos de ROS 2 no hace falta el
//     best_effort que exige PX4. El subscriber debe usar un QoS compatible.
//   - std_msgs/Float32 es un mensaje estándar con un solo campo: data.
//
// Ejecutar (y en otra terminal battery_subscriber, o: ros2 topic echo /vespo/battery_percent):
//   ros2 run px4_examples battery_publisher
// =============================================================================

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "px4_msgs/msg/battery_status.hpp"

using px4_msgs::msg::BatteryStatus;
using std_msgs::msg::Float32;

class BatteryPublisher : public rclcpp::Node
{
public:
  BatteryPublisher()
  : Node("battery_publisher")
  {
    // Publisher con QoS por defecto: rclcpp::QoS(10) = reliable, cola de 10.
    percent_pub_ = create_publisher<Float32>("/vespo/battery_percent", 10);

    auto px4_qos = rclcpp::QoS(1).best_effort().transient_local();
    battery_sub_ = create_subscription<BatteryStatus>(
      "/fmu/out/battery_status_v1", px4_qos,
      [this](const BatteryStatus::SharedPtr msg) {
        // remaining = -1 cuando PX4 aún no sabe: no publicamos basura.
        if (!msg->connected || msg->remaining < 0.0f) {
          return;
        }
        Float32 out;
        out.data = msg->remaining * 100.0f;   // 0..1 -> 0..100 %
        percent_pub_->publish(out);
      });

    RCLCPP_INFO(get_logger(), "Publicando el %% de batería en /vespo/battery_percent");
  }

private:
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Publisher<Float32>::SharedPtr percent_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryPublisher>());
  rclcpp::shutdown();
  return 0;
}
