"""Misión 5: el puente del lidar (Gazebo -> ROS) y el nodo cruzar_cueva.

    vespo up --sim --world cueva --custom x500_lidar_2d     # terminal 1
    ros2 launch x500_retos cueva.launch.py                    # terminal 2
    ros2 launch x500_retos cueva.launch.py paquete:=x500_soluciones

El lidar del x500_lidar_2d publica en Gazebo, no en ROS. ros_gz_bridge lo
traduce a sensor_msgs/LaserScan y aquí se renombra a /x500/scan.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topic de Gazebo: /world/<mundo>/model/<modelo>_<instancia>/link/link/sensor/lidar_2d_v2/scan
GZ_SCAN = '/world/cueva/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('paquete', default_value='x500_retos',
                              description='x500_retos (tu reto) o x500_soluciones'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='puente_lidar',
            # "[" = de Gazebo hacia ROS (solo en ese sentido)
            arguments=[f'{GZ_SCAN}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            remappings=[(GZ_SCAN, '/x500/scan')],
            output='screen'),
        Node(
            package=LaunchConfiguration('paquete'), executable='cruzar_cueva',
            output='screen', emulate_tty=True),
    ])
