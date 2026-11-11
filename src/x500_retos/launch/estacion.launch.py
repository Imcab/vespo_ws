"""Misión 7: cámara + detector_aruco + estacion_carga.

    vespo up --sim --world base_carga --custom x500_cam_abajo           # terminal 1
    ros2 launch x500_retos estacion.launch.py                             # terminal 2
    ros2 launch x500_retos estacion.launch.py paquete:=x500_soluciones

Para ver lo que ve el detector: ros2 run rqt_image_view rqt_image_view /x500/aruco/debug
"""
import os
import sys

from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(__file__))
from camara import camera_nodes, mission_node  # noqa: E402


def generate_launch_description():
    return LaunchDescription(camera_nodes('base_carga') + [mission_node('estacion_carga')])
