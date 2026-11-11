"""Misión 6: cámara + detector_aruco + caza_objetivo.

    vespo up --sim --world zona_bodegas --custom x500_cam_abajo         # terminal 1
    ros2 launch x500_retos caza.launch.py                                 # terminal 2
    ros2 launch x500_retos caza.launch.py paquete:=x500_soluciones

Para ver lo que ve el detector: ros2 run rqt_image_view rqt_image_view /x500/aruco/debug
"""
import os
import sys

from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(__file__))
from camara import camera_nodes, mission_node  # noqa: E402


def generate_launch_description():
    return LaunchDescription(camera_nodes('zona_bodegas') + [mission_node('caza_objetivo')])
