"""Piezas comunes de los launch del Nivel 3 (cámara de abajo + detector)."""
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_nodes(world):
    """Puente de la cámara del x500_cam_abajo (Gazebo -> ROS) y el detector.

    x500_cam_abajo y no el x500_mono_cam_down de PX4: su cámara de 1280x960 a
    30 Hz, pasada a ROS, puede llenar la RAM de una laptop (ver su model.sdf).
    """
    base = f'/world/{world}/model/x500_cam_abajo_0/link/camera_link/sensor/imager'
    return [
        DeclareLaunchArgument('paquete', default_value='x500_retos',
                              description='x500_retos (tu reto) o x500_soluciones'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='puente_camara',
            # "[" = de Gazebo hacia ROS
            arguments=[f'{base}/image@sensor_msgs/msg/Image[gz.msgs.Image',
                       f'{base}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            remappings=[(f'{base}/image', '/x500/camera/image'),
                        (f'{base}/camera_info', '/x500/camera/camera_info')],
            output='screen'),
        Node(package=LaunchConfiguration('paquete'), executable='detector_aruco',
             output='screen', emulate_tty=True),
    ]


def mission_node(name):
    return Node(package=LaunchConfiguration('paquete'), executable=name, output='screen', emulate_tty=True)
