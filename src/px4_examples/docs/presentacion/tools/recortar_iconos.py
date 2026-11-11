#!/usr/bin/env python3
"""Quita el fondo magenta de los renders *_icono_crudo.png y deja PNG transparentes.

    /data/vespo_ws/venv/bin/python tools/recortar_iconos.py     # después de renders_x500.py
"""
import pathlib

import cv2
import numpy as np

RENDER = pathlib.Path(__file__).resolve().parents[1] / 'img' / 'render'

for jpg in sorted(RENDER.glob('*_icono_crudo.png')):
    im = cv2.imread(str(jpg))
    b, g, r = (im[..., i].astype(int) for i in range(3))
    magenta = (r > 110) & (b > 110) & (g < 0.6 * np.minimum(r, b))
    alpha = cv2.morphologyEx(np.where(magenta, 0, 255).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ys, xs = np.where(alpha > 0)
    y0, y1, x0, x1 = ys.min() - 6, ys.max() + 6, xs.min() - 6, xs.max() + 6
    alpha = cv2.erode(alpha, np.ones((2, 2), np.uint8))
    # Bordes con tinte magenta (el rojo y el azul por encima del verde): a gris
    tinte = (np.minimum(r, b) - g) > 12
    im[tinte] = np.repeat(np.maximum(g[tinte], 0), 3).reshape(-1, 3).clip(0, 255)
    rgba = cv2.cvtColor(im, cv2.COLOR_BGR2BGRA)
    rgba[..., 3] = alpha
    out = jpg.with_name(jpg.name.replace('_crudo', ''))
    cv2.imwrite(str(out), rgba[max(y0, 0):y1, max(x0, 0):x1])
    jpg.unlink()
    print(out.name)
