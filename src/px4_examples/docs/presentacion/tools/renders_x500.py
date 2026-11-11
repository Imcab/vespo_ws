#!/usr/bin/env python3
"""Fotos del x500 en un "estudio" (pasto y cielo) para las presentaciones.

    /data/vespo_ws/venv/bin/python tools/renders_x500.py      # desde docs/presentacion

Deja JPGs en img/render/. Usa Gazebo sin ventana con su propio GZ_PARTITION
(no choca con una simulación abierta) y los modelos de PX4 y de x500_retos.
Es ligero (un solo modelo): funciona incluso con la laptop en batería.
"""
import base64
import json
import math
import os
import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = pathlib.Path(__file__).resolve().parents[1]
OUT = HERE / 'img' / 'render'
PX4_MODELS = '/data/PX4-Autopilot/Tools/simulation/gz/models'
RETOS_MODELS = '/data/vespo_ws/src/x500_retos/models'


def look_at(cam, target):
    """Pose 'x y z roll pitch yaw' de una cámara en `cam` que mira a `target`."""
    dx, dy, dz = (t - c for t, c in zip(target, cam))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    return f'{cam[0]} {cam[1]} {cam[2]} 0 {pitch:.4f} {yaw:.4f}'


# (nombre del archivo, modelo, z del dron, cámara, a dónde mira, hfov, ancho, alto)
SHOTS = [
    ('x500_tres_cuartos', 'x500_base', 0.25, (0.95, -0.95, 0.8), (0, 0, 0.3), 0.75, 1600, 1000),
    ('x500_arriba', 'x500_base', 0.25, (0.0, 0.0, 1.55), (0, 0.001, 0.3), 0.85, 1200, 1200),
    ('x500_frente', 'x500_base', 0.25, (0.0, 1.35, 0.45), (0, 0, 0.32), 0.85, 1600, 1000),
    ('x500_volando', 'x500_base', 2.0, (1.9, -1.9, 1.7), (0, 0, 1.95), 0.6, 1600, 1000),
    ('x500_lidar', 'x500_lidar_2d', 0.25, (0.95, -0.95, 0.85), (0, 0, 0.35), 0.75, 1600, 1000),
    ('x500_cam_abajo', 'x500_cam_abajo', 0.25, (0.9, -0.9, 0.3), (0, 0, 0.25), 0.8, 1600, 1000),
    # Vista de arriba sobre un piso magenta liso, para recortarla como ícono (x500_icono.png)
    ('x500_icono', 'x500_base', 0.25, (0.0, 0.0, 2.6), (0, 0.001, 0.3), 0.62, 1000, 1000),
    ('x500_lado_icono', 'x500_base', 0.25, (3.2, 0.0, 0.32), (0, 0, 0.32), 0.42, 1200, 700),
]

# Piso liso de un color que el dron no tiene, para quitarlo después
MAGENTA = ('<model name="piso"><static>true</static><link name="l"><visual name="v"><geometry><plane>'
           '<normal>0 0 1</normal><size>40 40</size></plane></geometry><material><ambient>1 0 1 1</ambient>'
           '<diffuse>1 0 1 1</diffuse><emissive>1 0 1 1</emissive></material></visual></link></model>')

PBR = ('<material><diffuse>1 1 1 1</diffuse><pbr><metal>'
       '<albedo_map>model://x500_texturas/pasto/pasto_color.jpg</albedo_map>'
       '<normal_map>model://x500_texturas/pasto/pasto_normal.jpg</normal_map>'
       '<metalness>0</metalness><roughness>0.9</roughness></metal></pbr></material>')


PISO = ('<model name="piso"><static>true</static><link name="l"><visual name="v"><geometry><mesh>'
        f'<uri>model://x500_mallas/plaza_show/pasto.obj</uri></mesh></geometry>{PBR}</visual></link></model>')


def world(shot, tag):
    name, model, z, cam, target, hfov, w, h = shot
    return f'''<?xml version="1.0"?><sdf version="1.9"><world name="estudio">
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<scene><ambient>0.55 0.55 0.6 1</ambient><background>{'1 0 1 1' if name.endswith('icono') else '0.7 0.8 0.95 1'}</background>{'' if name.endswith('icono') else '<sky></sky>'}<shadows>{'false' if name.endswith('icono') else 'true'}</shadows></scene>
<light type="directional" name="sol"><cast_shadows>true</cast_shadows><direction>-0.5 0.4 -0.8</direction>
  <diffuse>1 0.97 0.9 1</diffuse><specular>0.3 0.3 0.3 1</specular></light>
{MAGENTA if name.endswith('icono') else PISO}
<include><uri>model://{model}</uri><name>dron</name><pose>0 0 {z} 0 0 1.5708</pose><static>true</static></include>
<model name="cam"><static>true</static><link name="l"><sensor name="c" type="camera"><pose>{look_at(cam, target)}</pose>
  <topic>{tag}</topic><camera><horizontal_fov>{hfov}</horizontal_fov><image><width>{w}</width><height>{h}</height></image>
  <clip><near>0.05</near><far>200</far></clip></camera><always_on>1</always_on><update_rate>2</update_rate></sensor></link></model>
</world></sdf>'''


def render(shot):
    tag = f'render{os.getpid()}_{shot[0]}'
    sdf = OUT / f'__{shot[0]}.sdf'
    sdf.write_text(world(shot, tag))
    env = {**os.environ, 'GZ_PARTITION': tag,
           '__EGL_VENDOR_LIBRARY_FILENAMES': '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
           'GZ_SIM_RESOURCE_PATH': f'{RETOS_MODELS}:{PX4_MODELS}'}
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', str(sdf)], env=env,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        deadline = time.time() + 90
        while time.time() < deadline and server.poll() is None:
            try:
                r = subprocess.run(['gz', 'topic', '-e', '-n', '1', '--json-output', '-t', f'/{tag}'], env=env,
                                   capture_output=True, text=True, timeout=30)
            except subprocess.TimeoutExpired:
                continue
            if r.stdout.strip():
                m = json.loads(r.stdout.strip().splitlines()[0])
                img = np.frombuffer(base64.b64decode(m['data']), np.uint8).reshape(int(m['height']), int(m['width']), 3)
                bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                if shot[0].endswith('icono'):   # sin pérdida: el JPG mezcla el magenta en los bordes
                    cv2.imwrite(str(OUT / f'{shot[0]}_crudo.png'), bgr)
                else:
                    cv2.imwrite(str(OUT / f'{shot[0]}.jpg'), bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return True
        return False
    finally:
        os.killpg(server.pid, 9)
        sdf.unlink()


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    only = sys.argv[1:]
    for shot in SHOTS:
        if only and shot[0] not in only:
            continue
        print(shot[0], 'OK' if render(shot) else 'SIN IMAGEN', flush=True)
