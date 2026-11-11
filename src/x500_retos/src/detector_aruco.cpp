// =============================================================================
// detector_aruco (Nivel 3): los ojos del dron. Lee la cámara de abajo, busca
// marcadores ArUco con OpenCV y publica lo que vio en /x500/aruco.
//
// No sabe nada de PX4 ni de misiones: solo PERCEPCIÓN. Las misiones (caza_objetivo,
// estacion_carga) leen /x500/aruco y deciden. Si mañana cambian la cámara o el
// detector, las misiones no se tocan.
//
// Lo arrancan los launch del Nivel 3 (caza.launch.py, estacion.launch.py),
// junto con el puente de la cámara (ros_gz_bridge):
//   /x500/camera/image        sensor_msgs/Image       (640 x 480, 15 Hz)
//   /x500/camera/camera_info  sensor_msgs/CameraInfo  (fx, fy, cx, cy)
// Publica:
//   /x500/aruco               x500_retos/ArucoArray   (id, centro, x_norm, y_norm)
//   /x500/aruco/debug         sensor_msgs/Image       (la imagen con los marcadores
//                             dibujados; solo si alguien la ve:
//                             ros2 run rqt_image_view rqt_image_view)
//
// Familia DICT_5X5_250 (la de tools/aruco y la de TMR). OpenCV 4.6, el de
// Ubuntu 24.04: la API es cv::aruco::detectMarkers (en 4.7+ cambió a
// cv::aruco::ArucoDetector).
//
// TU PARTE: el TODO en on_image(): detectar y llenar los mensajes. Mientras
// no lo hagas, publica listas vacías (las misiones simplemente no ven nada).
// =============================================================================

#include <memory>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "x500_retos/msg/aruco_array.hpp"

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using x500_retos::msg::Aruco;
using x500_retos::msg::ArucoArray;

class DetectorAruco : public rclcpp::Node
{
public:
  DetectorAruco()
  : Node("detector_aruco")
  {
    max_rate_ = declare_parameter("max_rate", 15.0);   // Hz: no hace falta procesar los 30

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    params_ = cv::aruco::DetectorParameters::create();

    info_sub_ = create_subscription<CameraInfo>(
      "/x500/camera/camera_info", rclcpp::SensorDataQoS(),
      [this](const CameraInfo::SharedPtr msg) {info_ = msg;});
    image_sub_ = create_subscription<Image>(
      "/x500/camera/image", rclcpp::SensorDataQoS(),
      [this](const Image::SharedPtr msg) {on_image(msg);});
    pub_ = create_publisher<ArucoArray>("/x500/aruco", 10);
    debug_pub_ = create_publisher<Image>("/x500/aruco/debug", 1);
    RCLCPP_INFO(get_logger(), "detector_aruco: esperando la cámara en /x500/camera/image");
  }

private:
  void on_image(const Image::SharedPtr msg)
  {
    // La calibración (fx, fy, cx, cy) viene en camera_info: sin ella no se
    // puede pasar de píxeles a ángulos.
    if (!info_) {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp);
    if (last_.nanoseconds() > 0 && (stamp - last_).seconds() < 1.0 / max_rate_) {
      return;
    }
    last_ = stamp;

    // De mensaje de ROS a imagen de OpenCV, ya en gris (el detector usa gris).
    cv_bridge::CvImageConstPtr gray = cv_bridge::toCvShare(msg, "mono8");
    const double fx = info_->k[0], cx = info_->k[2], fy = info_->k[4], cy = info_->k[5];

    ArucoArray out;
    out.header = msg->header;
    out.width = msg->width;
    out.height = msg->height;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // TODO: detectar y llenar out.markers.
    //   1. cv::aruco::detectMarkers(gray->image, dictionary_, corners, ids, params_);
    //      deja en ids los números y en corners las 4 esquinas (en píxeles)
    //      de cada marcador encontrado.
    //   2. Para cada uno, un Aruco con:
    //        id      = ids[i]
    //        u, v    = el centro: el promedio de las 4 esquinas
    //        x_norm  = (u - cx) / fx      y_norm = (v - cy) / fy
    //        size_px = el promedio de los 4 lados (cv::norm(a - b) es la
    //                  distancia entre dos esquinas)
    //      y out.markers.push_back(m).
    (void)fx; (void)fy; (void)cx; (void)cy;
    pub_->publish(out);

    if (!out.markers.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "viendo %zu marcador(es), el primero: id %d",
                           out.markers.size(), out.markers[0].id);
    }
    // Imagen de depuración: solo si alguien la está viendo (dibujar cuesta).
    if (debug_pub_->get_subscription_count() > 0) {
      cv::Mat color;
      cv::cvtColor(gray->image, color, cv::COLOR_GRAY2BGR);
      if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(color, corners, ids);
      }
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", color).toImageMsg());
    }
  }

  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Publisher<ArucoArray>::SharedPtr pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  CameraInfo::SharedPtr info_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
  double max_rate_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectorAruco>());
  rclcpp::shutdown();
  return 0;
}
