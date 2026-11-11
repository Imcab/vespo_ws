#!/usr/bin/env python3
"""Genera las texturas ArUco de los mundos del Nivel 3.

    /data/vespo_ws/venv/bin/python tools/make_aruco.py     # el venv de tools/aruco trae OpenCV

Familia DICT_5X5_250, la misma de tools/aruco y del reglamento de TMR. Cada
PNG lleva un margen blanco de una celda (la "zona quieta"): sin él, sobre un
fondo oscuro el detector no encuentra la orilla del marcador.

    id  7   la camioneta sospechosa (misión 6)
    id  3, 12, 25   coches estacionados: señuelos (misión 6)
    id 42   la estación de carga (misión 7)

Los nombres de archivo son únicos (aruco_<id>.png) a propósito: ogre2 confunde
texturas que se llaman igual aunque estén en carpetas distintas.
"""
import pathlib

import cv2
import numpy as np

OUT = pathlib.Path(__file__).resolve().parents[1] / 'models' / 'x500_aruco'
IDS = [3, 7, 12, 25, 42]
CELL = 100   # px por celda: 5x5 + borde negro (1) = 7 celdas, + margen blanco = 9


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    for marker_id in IDS:
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 7 * CELL, borderBits=1)
        img = np.full((9 * CELL, 9 * CELL), 255, np.uint8)
        img[CELL:8 * CELL, CELL:8 * CELL] = marker
        cv2.imwrite(str(OUT / f'aruco_{marker_id}.png'), img)
        print('aruco', marker_id)


if __name__ == '__main__':
    main()
