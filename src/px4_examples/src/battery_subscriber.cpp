// =============================================================================
// battery_subscriber: recibe el valor que publica battery_publisher en
// /vespo/battery_percent y lo muestra como una barra.
//
// No usa px4_msgs para nada: solo std_msgs/Float32. Así se ve cómo dos nodos
// nuestros se pasan un valor por un topic.
//
// Ejecutar (con battery_publisher corriendo en otra terminal):
//   ros2 run px4_examples battery_subscriber
//
// También se puede mandar un valor A MANO, sin PX4, para probarlo:
//   ros2 topic pub /vespo/battery_percent std_msgs/msg/Float32 "{data: 15.0}"
// =============================================================================

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"

using std_msgs::msg::Float32;

class BatterySubscriber : public rclcpp::Node
{
public:
  BatterySubscriber()
  : Node("battery_subscriber")
  {
    low_percent_ = declare_parameter("low_percent", 30.0);

    // Mismo QoS que el publisher (reliable, por defecto).
    percent_sub_ = create_subscription<Float32>(
      "/vespo/battery_percent", 10,
      [this](const Float32::SharedPtr msg) {on_percent(msg->data);});

    RCLCPP_INFO(get_logger(), "Esperando valores en /vespo/battery_percent...");
  }

private:
  void on_percent(float percent)
  {
    // Barra de 20 caracteres: cada # vale 5 %.
    const int filled = static_cast<int>(percent / 5.0f + 0.5f);
    std::string bar;
    for (int i = 0; i < 20; i++) {
      bar += i < filled ? "#" : ".";
    }

    if (percent < low_percent_) {
      RCLCPP_WARN(get_logger(), "[%s] %5.1f %%  BAJA", bar.c_str(), percent);
    } else {
      RCLCPP_INFO(get_logger(), "[%s] %5.1f %%", bar.c_str(), percent);
    }
  }

  rclcpp::Subscription<Float32>::SharedPtr percent_sub_;
  double low_percent_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatterySubscriber>());
  rclcpp::shutdown();
  return 0;
}
