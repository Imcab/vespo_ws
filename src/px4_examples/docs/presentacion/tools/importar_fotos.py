#!/usr/bin/env python3
"""Copia a img/px4docs/ las imágenes de la documentación de PX4 que usan las presentaciones.

    ~/.cache/x500_probar/venv/bin/python tools/importar_fotos.py     # necesita pillow y cairosvg

La documentación de PX4 (docs/ del repo PX4-Autopilot, https://docs.px4.io) tiene
licencia CC BY 4.0: se pueden usar citando la fuente (ver img/px4docs/CREDITOS.md).
"""
import pathlib

import cairosvg
from PIL import Image

A = pathlib.Path('/data/PX4-Autopilot/docs/assets')
OUT = pathlib.Path(__file__).resolve().parents[1] / 'img' / 'px4docs'

FOTOS = {   # destino: origen (relativo a docs/assets)
    'x500v2_armado.jpg': 'airframes/multicopter/x500_v2_holybro_pixhawk5x/finalized_x500v2_kit.png',
    'x500v2_kit.jpg': 'airframes/multicopter/x500_v2_holybro_pixhawk5x/x500_v2_whats_inside.png',
    'x500_kit_etiquetado.jpg': 'airframes/multicopter/x500_holybro_pixhawk4/whats_inside_x500_labeled.jpg',
    'x500_armado.jpg': 'airframes/multicopter/x500_holybro_pixhawk4/X500_assembled_frame.jpg',
    'pixhawk6c.jpg': 'flight_controller/pixhawk6c/pixhawk6c_hero_upright.png',
    'pixhawk6c_frente.jpg': 'flight_controller/pixhawk6c/pixhawk6c_vehicle_front1.jpg',
    'gps_m9n.jpg': 'hardware/gps/holybro_m9n_gps.jpg',
    'qgc_fly_view.jpg': 'concepts/qgc_fly_view.png',
    'qgc_motores.png': 'config/actuators/qgc_actuators_mc_geometry_marked.png',
    'qgc_airframe.jpg': 'airframes/multicopter/x500_v2_holybro_pixhawk5x/x500v2_airframe_qgc.png',
    'gazebo_x500.png': 'simulation/gazebo/vehicles/x500.png',
    'telemetria_conexion.png': 'assembly/telemetry_connections.png',
}
SVGS = {
    'xrce_dds.png': 'middleware/xrce_dds/architecture_xrce-dds_ros2.svg',
    'px4_arquitectura.png': 'diagrams/PX4_Architecture.svg',
}


def guardar(img, destino):
    img = img.convert('RGBA')
    fondo = Image.new('RGBA', img.size, 'white')
    fondo.alpha_composite(img)
    img = fondo.convert('RGB')
    img.thumbnail((1600, 1600))
    if destino.suffix == '.jpg':
        img.save(destino, quality=85)
    else:
        img.save(destino, optimize=True)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for destino, origen in FOTOS.items():
        guardar(Image.open(A / origen), OUT / destino)
        print(destino)
    for destino, origen in SVGS.items():
        tmp = OUT / ('__' + destino)
        cairosvg.svg2png(url=str(A / origen), write_to=str(tmp), output_width=1800)
        guardar(Image.open(tmp), OUT / destino)
        tmp.unlink()
        print(destino)


if __name__ == '__main__':
    main()
