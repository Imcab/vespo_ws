#!/usr/bin/env python3
"""Toma fotos de un mundo sin abrir Gazebo (para docs/img y para revisar cambios).

    /data/vespo_ws/venv/bin/python tools/render_world.py MUNDO SALIDA nombre:x,y,z,roll,pitch,yaw[:hfov] ...

  MUNDO   nombre de un mundo de worlds/ (p. ej. plaza_show)
  SALIDA  carpeta donde quedan <nombre>.png
  cámara  pose en el marco de Gazebo (ENU), yaw en radianes; hfov opcional (1.2)

Agrega al mundo una cámara por foto y un x500 en el punto de despegue (para
ver la escala), corre Gazebo headless con su propio GZ_PARTITION (no choca con
una simulación abierta) y guarda una imagen de cada cámara. Usa el venv del
repo porque necesita OpenCV y numpy.

Ojo: gz no saca imágenes de un mundo con <actor> animados (se congela el render).
"""
import base64
import json
import os
import pathlib
import re
import subprocess
import sys
import time

import cv2
import numpy as np

PKG = pathlib.Path(__file__).resolve().parents[1]
PX4_MODELS = pathlib.Path('/data/PX4-Autopilot/Tools/simulation/gz/models')
NVIDIA_EGL = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'


def main():
    world, out, cams = sys.argv[1], pathlib.Path(sys.argv[2]).resolve(), sys.argv[3:]
    out.mkdir(parents=True, exist_ok=True)
    tag = f'render{os.getpid()}'
    sensors = []
    for c in cams:
        parts = c.split(':')
        name, cam_pose = parts[0], parts[1].replace(',', ' ')
        hfov = parts[2] if len(parts) > 2 else '1.2'
        sensors.append(
            f'<sensor name="{name}" type="camera"><pose>{cam_pose}</pose><topic>{tag}/{name}</topic>'
            f'<camera><horizontal_fov>{hfov}</horizontal_fov><image><width>1280</width><height>720</height>'
            f'</image><clip><near>0.1</near><far>1500</far></clip></camera>'
            f'<always_on>1</always_on><update_rate>2</update_rate></sensor>')
    extra = (f'<model name="__camaras"><static>true</static><link name="l">{"".join(sensors)}</link></model>'
             '<include><uri>model://x500_base</uri><name>dron</name>'
             '<pose>0 0 0.25 0 0 1.5708</pose><static>true</static></include>')
    # Un mundo de render necesita sus propios sistemas (el de PX4 los trae en server.config).
    systems = ('<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
               '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>'
               '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
               '<render_engine>ogre2</render_engine></plugin>')
    sdf = (PKG / 'worlds' / world / f'{world}.sdf').read_text()
    if '<plugin' not in sdf:   # los mundos con cámara ya traen sus sistemas
        sdf = re.sub(r'(<world[^>]*>)', lambda m: m.group(1) + systems, sdf, count=1)
    sdf = sdf.replace('</world>', extra + '</world>')
    tmp = out / f'__{world}.sdf'
    tmp.write_text(sdf)
    env = {**os.environ, 'GZ_PARTITION': tag, '__EGL_VENDOR_LIBRARY_FILENAMES': NVIDIA_EGL,
           'GZ_SIM_RESOURCE_PATH': ':'.join([str(PKG / 'models'), str(PX4_MODELS)])}
    log = open(out / f'__{world}.log', 'w')
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', os.environ.get('GZ_VERBOSE', '1'), str(tmp)], env=env,
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.time() + float(os.environ.get('TMO', 150))
        for c in cams:
            name = c.split(':')[0]
            got = False
            while not got and time.time() < deadline and server.poll() is None:
                try:
                    r = subprocess.run(['gz', 'topic', '-e', '-n', '1', '--json-output', '-t', f'/{tag}/{name}'],
                                       env=env, capture_output=True, text=True,
                                       timeout=max(5, deadline - time.time()))
                except subprocess.TimeoutExpired:
                    break
                if r.stdout.strip():
                    m = json.loads(r.stdout.strip().splitlines()[0])
                    img = np.frombuffer(base64.b64decode(m['data']), np.uint8)
                    img = img.reshape(int(m['height']), int(m['width']), 3)
                    cv2.imwrite(str(out / f'{name}.png'), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    got = True
            print(name, 'OK' if got else 'SIN IMAGEN')
    finally:
        os.killpg(server.pid, 9)
        log.close()
        tmp.unlink()
    errors = [l for l in (out / f'__{world}.log').read_text().splitlines() if 'Err]' in l]
    print('\n'.join(dict.fromkeys(errors))[:3000])


if __name__ == '__main__':
    main()
