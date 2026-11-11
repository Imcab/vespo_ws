#!/usr/bin/env python3
"""Genera los mundos de Gazebo de x500_retos.

    python3 tools/build_worlds.py          # reescribe worlds/ y models/x500_mallas/

Cada mundo es un .sdf en worlds/<mundo>/ con su spawn.yaml. Este script
escribe dos cosas:

  - Mallas .obj con UVs a escala real (models/x500_mallas/<mundo>/). Una
    textura de 1 m no puede ir en un <plane> de Gazebo, porque el plano estira
    la imagen completa sobre todo el tamaño; la malla la repite cada `tile` m.
  - El .sdf del mundo: piso, escenografía propia (balizas, puente, farmacia...)
    y los modelos de Fuel copiados en models/ (casas, árboles, coches...).

Convención de todos los mundos: el dron aparece en (0, 0) mirando al norte
(yaw 90° en Gazebo). Gazebo usa ENU (x = este, y = norte, z = arriba) y PX4
usa NED con origen donde arrancó, así que aquí:

    PX4 x (norte) = Gazebo y        PX4 y (este) = Gazebo x      PX4 z = -Gazebo z

Las colisiones son cajas (box), no mallas: el motor de física de Gazebo
(dartsim) no choca de forma confiable contra mallas.
"""
import math
import pathlib
import random
import re

PKG = pathlib.Path(__file__).resolve().parents[1]
MALLAS = PKG / 'models' / 'x500_mallas'
WORLDS = PKG / 'worlds'

# Ubicación geográfica de todos los mundos: la de los mundos de PX4. El campo
# magnético de abajo corresponde a este lugar; si se cambia uno sin el otro el
# chequeo de brújula de PX4 falla.
LAT0, LON0 = 47.397971057728974, 8.546163739800146


# =============================================================================
# Mallas .obj
# =============================================================================

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = math.sqrt(sum(c * c for c in a)) or 1.0
    return tuple(c / n for c in a)


class Obj:
    """Malla de cuadriláteros con normales y UVs. Un material por archivo."""

    def __init__(self):
        self.v, self.t, self.n, self.f = [], [], [], []

    def quad(self, pts, uvs):
        """pts en sentido antihorario visto desde el lado de afuera."""
        normal = _unit(_cross(_sub(pts[1], pts[0]), _sub(pts[2], pts[0])))
        base_v, base_t = len(self.v), len(self.t)
        self.v += pts
        self.t += uvs
        self.n.append(normal)
        ni = len(self.n)
        self.f.append([(base_v + i + 1, base_t + i + 1, ni) for i in range(len(pts))])

    def quad_facing(self, pts, uvs, toward, outward=False):
        """Como quad(), pero voltea el orden si hace falta para que la cara
        mire hacia el punto `toward` (o en contra con outward=True). Importa:
        Gazebo no dibuja la cara de atrás y el lidar tampoco la ve."""
        normal = _cross(_sub(pts[1], pts[0]), _sub(pts[2], pts[0]))
        cx = sum(p[0] for p in pts) / 4
        cy = sum(p[1] for p in pts) / 4
        cz = sum(p[2] for p in pts) / 4
        to = (toward[0] - cx, toward[1] - cy, toward[2] - cz)
        facing = sum(a * b for a, b in zip(normal, to)) > 0
        if facing == outward:
            pts, uvs = pts[::-1], uvs[::-1]
        self.quad(pts, uvs)

    def ring_road(self, cx, cy, r_in, r_out, z, segments=120):
        """Anillo (calle circular): la textura de la calle cubre el ancho y
        sus líneas siguen la curva."""
        width = r_out - r_in
        for i in range(segments):
            a0, a1 = 2 * math.pi * i / segments, 2 * math.pi * (i + 1) / segments
            s0, s1 = a0 * (r_in + r_out) / 2 / width, a1 * (r_in + r_out) / 2 / width
            pts = [(cx + r_in * math.cos(a0), cy + r_in * math.sin(a0), z),
                   (cx + r_out * math.cos(a0), cy + r_out * math.sin(a0), z),
                   (cx + r_out * math.cos(a1), cy + r_out * math.sin(a1), z),
                   (cx + r_in * math.cos(a1), cy + r_in * math.sin(a1), z)]
            self.quad_facing(pts, [(0, s0), (1, s0), (1, s1), (0, s1)], (cx, cy, z + 10))

    def plane(self, x0, y0, x1, y1, z, tile, along_x=False):
        """Rectángulo horizontal. along_x pone el eje v de la textura sobre x
        (para que las líneas de la calle corran a lo largo de una calle este-oeste)."""
        def uv(x, y):
            return (y / tile, x / tile) if along_x else (x / tile, y / tile)
        pts = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
        self.quad(pts, [uv(p[0], p[1]) for p in pts])

    def road(self, x0, y0, x1, y1, z, along_x):
        """Calle: la textura completa cubre el ancho y se repite a lo largo."""
        width = (y1 - y0) if along_x else (x1 - x0)
        pts = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
        if along_x:
            uvs = [((p[1] - y0) / width, p[0] / width) for p in pts]
        else:
            uvs = [((p[0] - x0) / width, p[1] / width) for p in pts]
        self.quad(pts, uvs)

    def box(self, cx, cy, cz, sx, sy, sz, tile, bottom=False):
        x0, x1, y0, y1, z0, z1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, cz - sz / 2, cz + sz / 2
        t = tile
        self.quad([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
                  [(x0 / t, y0 / t), (x1 / t, y0 / t), (x1 / t, y1 / t), (x0 / t, y1 / t)])
        if bottom:
            self.quad([(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)],
                      [(x0 / t, y1 / t), (x1 / t, y1 / t), (x1 / t, y0 / t), (x0 / t, y0 / t)])
        self.quad([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
                  [(x0 / t, z0 / t), (x1 / t, z0 / t), (x1 / t, z1 / t), (x0 / t, z1 / t)])
        self.quad([(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)],
                  [(x1 / t, z0 / t), (x0 / t, z0 / t), (x0 / t, z1 / t), (x1 / t, z1 / t)])
        self.quad([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
                  [(y0 / t, z0 / t), (y1 / t, z0 / t), (y1 / t, z1 / t), (y0 / t, z1 / t)])
        self.quad([(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)],
                  [(y1 / t, z0 / t), (y0 / t, z0 / t), (y0 / t, z1 / t), (y1 / t, z1 / t)])

    def cylinder(self, cx, cy, z0, z1, r, tile, sides=40, top=True):
        circ = 2 * math.pi * r
        for i in range(sides):
            a0, a1 = 2 * math.pi * i / sides, 2 * math.pi * (i + 1) / sides
            p0 = (cx + r * math.cos(a0), cy + r * math.sin(a0))
            p1 = (cx + r * math.cos(a1), cy + r * math.sin(a1))
            u0, u1 = circ * i / sides / tile, circ * (i + 1) / sides / tile
            self.quad([(p0[0], p0[1], z0), (p1[0], p1[1], z0), (p1[0], p1[1], z1), (p0[0], p0[1], z1)],
                      [(u0, z0 / tile), (u1, z0 / tile), (u1, z1 / tile), (u0, z1 / tile)])
            if top:
                pts = [(cx, cy, z1), (p0[0], p0[1], z1), (p1[0], p1[1], z1)]
                self.quad(pts, [(p[0] / tile, p[1] / tile) for p in pts])

    def profile(self, xz, y0, y1, tile):
        """Terreno: un perfil (x, z) extruido de y0 a y1."""
        dist = 0.0
        for (xa, za), (xb, zb) in zip(xz, xz[1:]):
            seg = math.hypot(xb - xa, zb - za)
            ua, ub = dist / tile, (dist + seg) / tile
            self.quad([(xa, y0, za), (xb, y0, zb), (xb, y1, zb), (xa, y1, za)],
                      [(ua, y0 / tile), (ub, y0 / tile), (ub, y1 / tile), (ua, y1 / tile)])
            dist += seg

    def save(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        # Cada malla con su propio material (nombre único): Gazebo guarda los
        # materiales por nombre y, sin esto, todas las mallas comparten el
        # primero que cargó (todas salían con la textura del pasto).
        mat = f'{path.parent.name}_{path.stem}'
        path.with_suffix('.mtl').write_text(f'newmtl {mat}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\n')
        lines = ['# generado por tools/build_worlds.py', f'mtllib {path.stem}.mtl', f'usemtl {mat}']
        lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for x, y, z in self.v]
        lines += [f'vt {u:.4f} {v:.4f}' for u, v in self.t]
        lines += [f'vn {x:.4f} {y:.4f} {z:.4f}' for x, y, z in self.n]
        lines += ['f ' + ' '.join(f'{a}/{b}/{c}' for a, b, c in face) for face in self.f]
        path.write_text('\n'.join(lines) + '\n')


# =============================================================================
# Piezas de SDF
# =============================================================================

def tex(name, tint='1 1 1 1'):
    """Material PBR de models/x500_texturas/<name> (texturas de ambientCG)."""
    base = f'model://x500_texturas/{name}'
    return (f'<material><ambient>{tint}</ambient><diffuse>{tint}</diffuse><specular>0.05 0.05 0.05 1</specular>'
            f'<pbr><metal><albedo_map>{base}/{name}_color.jpg</albedo_map><normal_map>{base}/{name}_normal.jpg</normal_map>'
            f'<roughness_map>{base}/{name}_roughness.jpg</roughness_map><metalness>0</metalness></metal></pbr></material>')


def color(rgb, glow=0.0, rough=0.6, metal=0.0):
    """Material liso. glow > 0 lo hace emisivo (brilla aunque no le dé luz)."""
    r, g, b = rgb
    def lit(c):
        return min(1.0, c * glow)
    emissive = f'<emissive>{lit(r):.3f} {lit(g):.3f} {lit(b):.3f} 1</emissive>' if glow else ''
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.2 0.2 0.2 1</specular>{emissive}'
            f'<pbr><metal><roughness>{rough}</roughness><metalness>{metal}</metalness></metal></pbr></material>')


def box(sx, sy, sz):
    return f'<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'


def cyl(r, length):
    return f'<cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder>'


def mesh(world, name):
    return f'<mesh><uri>model://x500_mallas/{world}/{name}.obj</uri></mesh>'


def pose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    return f'<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>'


class Link:
    """Un link estático con visuales y colisiones."""

    def __init__(self, name):
        self.name, self.items, self.count = name, [], 0

    def _id(self, prefix):
        self.count += 1
        return f'{prefix}_{self.count}'

    def visual(self, geom, mat, at='', transparency=0.0, shadows=True):
        extra = f'<transparency>{transparency}</transparency>' if transparency else ''
        extra += '' if shadows else '<cast_shadows>false</cast_shadows>'
        self.items.append(f'<visual name="{self._id("v")}">{at}<geometry>{geom}</geometry>{mat}{extra}</visual>')

    def collision(self, geom, at=''):
        self.items.append(f'<collision name="{self._id("c")}">{at}<geometry>{geom}</geometry></collision>')

    def solid(self, geom, mat, at='', **kw):
        self.visual(geom, mat, at, **kw)
        self.collision(geom, at)

    def light(self, xml):
        self.items.append(xml)

    def xml(self):
        return f'<link name="{self.name}">' + '\n'.join(self.items) + '</link>'


def static_model(name, links, at=''):
    return (f'<model name="{name}"><static>true</static>{at}\n'
            + '\n'.join(link.xml() for link in links) + '\n</model>')


def include(model, name, x, y, z=0.0, yaw=0.0):
    return f'<include><uri>model://{model}</uri><name>{name}</name>{pose(x, y, z, 0, 0, yaw)}</include>'


def point_light(name, x, y, z, rgb, rng=12.0, intensity=1.0, shadows=False):
    r, g, b = rgb
    return (f'<light type="point" name="{name}">{pose(x, y, z)}<diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>{r * 0.3:.2f} {g * 0.3:.2f} {b * 0.3:.2f} 1</specular><intensity>{intensity}</intensity>'
            f'<attenuation><range>{rng}</range><constant>0.2</constant><linear>0.05</linear>'
            f'<quadratic>0.01</quadratic></attenuation><cast_shadows>{str(shadows).lower()}</cast_shadows></light>')


def spot_light(name, x, y, z, direction, rgb, rng=25.0, intensity=1.0, angle=0.5):
    r, g, b = rgb
    dx, dy, dz = direction
    return (f'<light type="spot" name="{name}">{pose(x, y, z)}<diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.2 0.2 0.2 1</specular><intensity>{intensity}</intensity>'
            f'<direction>{dx} {dy} {dz}</direction>'
            f'<attenuation><range>{rng}</range><constant>0.2</constant><linear>0.02</linear>'
            f'<quadratic>0.002</quadratic></attenuation>'
            f'<spot><inner_angle>{angle * 0.6:.3f}</inner_angle><outer_angle>{angle:.3f}</outer_angle>'
            f'<falloff>1.0</falloff></spot><cast_shadows>false</cast_shadows></light>')


def sun(direction, rgb, intensity=1.0):
    dx, dy, dz = direction
    r, g, b = rgb
    return (f'<light type="directional" name="sol"><cast_shadows>true</cast_shadows>{pose(0, 0, 60)}'
            f'<diffuse>{r} {g} {b} 1</diffuse><specular>0.25 0.25 0.25 1</specular>'
            f'<intensity>{intensity}</intensity><direction>{dx} {dy} {dz}</direction></light>')


# Los sistemas de Gazebo que carga el server.config de PX4, MENOS GstCameraSystem.
# Ese plugin toma la primera cámara del mundo y la manda como video H.264 por
# UDP; sin CUDA codifica por software, no alcanza, y su appsrc (sin límite)
# acumula TODAS las imágenes: Gazebo creció a 13 GB y trabó la computadora.
# Los mundos con cámara declaran sus sistemas con esta lista; al declarar
# alguno, Gazebo ya no usa el server.config de PX4.
CAMERA_WORLD_SYSTEMS = """
    <!-- Sistemas declarados a mano: los del server.config de PX4 sin
         GstCameraSystem (su video H.264 acumula imágenes en memoria hasta
         llenar la RAM). Ver CAMERA_WORLD_SYSTEMS en tools/build_worlds.py. -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>"""


def world_sdf(name, description, scene, body, systems=None):
    plugins = systems or """
    <!-- Sin <plugin> a propósito: si el mundo declara alguno, Gazebo ignora el
         server.config de PX4 (física, IMU, barómetro, GPS, cámaras). -->"""
    return f'''<?xml version="1.0" ?>
<!-- {description}

     Generado por tools/build_worlds.py: los cambios a mano se pierden al
     regenerar. Se corre con  vespo up --sim --world {name}
-->
<sdf version="1.9">
  <world name="{name}">

{plugins}
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{LAT0}</latitude_deg>
      <longitude_deg>{LON0}</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    {scene}

{body}
  </world>
</sdf>
'''


SCENE_DAY = ('<scene><ambient>0.5 0.5 0.55 1</ambient><background>0.62 0.75 0.92 1</background>'
             '<sky></sky><shadows>true</shadows><grid>false</grid></scene>')
SCENE_DUSK = ('<scene><ambient>0.16 0.14 0.24 1</ambient><background>0.07 0.06 0.16 1</background>'
              '<shadows>true</shadows><grid>false</grid></scene>')


def spawn_yaml(world, text):
    (WORLDS / world / 'spawn.yaml').write_text(
        f'# Donde `vespo simulate --world {world}` pone el dron (a menos que se pase --pose).\n'
        '# yaw_deg 90 = mirando al norte, así el heading de PX4 arranca en 0.\n'
        f'# {text}\n'
        'x: 0.0\ny: 0.0\nz: 0.3\nyaw_deg: 90\n')


def helipad(link, x=0.0, y=0.0, ring=(1.0, 1.0, 1.0), glow=0.0):
    """Plataforma de despegue con "H" (solo visual, pegada al piso)."""
    link.visual(cyl(1.35, 0.02), color(ring, glow), pose(x, y, 0.03), shadows=False)
    link.visual(cyl(1.2, 0.02), color((0.12, 0.12, 0.14)), pose(x, y, 0.035), shadows=False)
    white = color((0.95, 0.95, 0.95), glow)
    for dx in (-0.4, 0.4):
        link.visual(box(0.16, 1.1, 0.01), white, pose(x + dx, y, 0.05), shadows=False)
    link.visual(box(0.8, 0.16, 0.01), white, pose(x, y, 0.05), shadows=False)


def lamp_post(name, x, y, toward, light=True, rgb=(1.0, 0.82, 0.55), rng=20.0, intensity=1.5, z=0.0):
    """Poste de luz de Fuel con el brazo apuntando hacia `toward` (x, y).

    El brazo del modelo apunta a -y y la lámpara queda a ~3 m del poste y
    ~8.9 m de alto."""
    dx, dy = toward[0] - x, toward[1] - y
    yaw = math.atan2(dx, -dy)
    out = include('lamp_post', name, x, y, z, yaw)
    if light:
        hx, hy = x + 3.0 * math.sin(yaw), y - 3.0 * math.cos(yaw)
        out += '\n' + point_light(f'{name}_luz', hx, hy, z + 8.4, rgb, rng, intensity)
    return out


def scatter(seed, n, region, keep_out, min_gap=5.0):
    """Puntos al azar (pero siempre los mismos) dentro de region, fuera de keep_out."""
    rnd = random.Random(seed)
    pts, tries = [], 0
    x0, y0, x1, y1 = region
    while len(pts) < n and tries < 20000:
        tries += 1
        p = (rnd.uniform(x0, x1), rnd.uniform(y0, y1))
        if any(k(p) for k in keep_out):
            continue
        if any(math.hypot(p[0] - q[0], p[1] - q[1]) < min_gap for q in pts):
            continue
        pts.append(p)
    return pts, rnd


def rect(x0, y0, x1, y1):
    return lambda p: x0 <= p[0] <= x1 and y0 <= p[1] <= y1


def circle(cx, cy, r):
    return lambda p: math.hypot(p[0] - cx, p[1] - cy) <= r


def trees(seed, n, region, keep_out, min_gap=6.0, pine_ratio=0.4):
    pts, rnd = scatter(seed, n, region, keep_out, min_gap)
    out = []
    for i, (x, y) in enumerate(pts):
        model = 'pine_tree' if rnd.random() < pine_ratio else 'oak_tree'
        out.append(include(model, f'arbol_{seed}_{i}', x, y, 0.0, rnd.uniform(0, 2 * math.pi)))
    return '\n'.join(out)


# =============================================================================
# Mundo 1: plaza_show (misión 1, la estrella; después la geocerca)
# =============================================================================

def star_vertices(points, radius, inner_ratio):
    """Vértices de la estrella en ENU, empezando por el pico norte.

    Es la misma cuenta que tiene que hacer la misión 1, pero en NED:
    pico k en el ángulo 2*pi*k/N medido desde el norte, y entre picos un
    vértice interior a radius*inner_ratio."""
    out = []
    for k in range(2 * points):
        a = math.pi * k / points
        r = radius if k % 2 == 0 else radius * inner_ratio
        out.append((r * math.sin(a), r * math.cos(a)))   # (este, norte)
    return out


# Geocerca de la misión 4 (ENU): la caja donde el dron puede volar.
FENCE = dict(x0=-10.0, x1=10.0, y0=-6.0, y1=14.0, top=8.0)


def build_plaza_show(concert=False):
    """concert=False: plaza_show (misión 1). concert=True: plaza_concierto
    (misión 4), la misma plaza sin balizas y con la geocerca dibujada."""
    w = 'plaza_concierto' if concert else 'plaza_show'
    R, INNER, ALT = 5.0, 0.382, 6.0      # los valores por defecto de la misión 1
    PL = 22.0                             # media plaza

    piso = Obj()
    piso.plane(-130, -130, 130, 130, 0.0, 4.0)
    piso.save(MALLAS / w / 'pasto.obj')
    plaza = Obj()
    plaza.plane(-PL, -PL, PL, PL, 0.01, 2.5)
    plaza.save(MALLAS / w / 'adoquin.obj')
    calle = Obj()
    calle.road(-130, -40, 130, -32, 0.012, along_x=True)
    calle.save(MALLAS / w / 'calle.obj')
    banqueta = Obj()
    banqueta.box(0, -30.75, 0.06, 260, 2.5, 0.12, 2.0)
    banqueta.box(0, -41.25, 0.06, 260, 2.5, 0.12, 2.0)
    for s in (-1, 1):   # orillas de la plaza
        banqueta.box(0, s * (PL + 0.25), 0.08, 2 * PL + 1, 0.5, 0.16, 1.0)
        banqueta.box(s * (PL + 0.25), 0, 0.08, 0.5, 2 * PL, 0.16, 1.0)
    banqueta.save(MALLAS / w / 'banqueta.obj')
    tarima = Obj()
    tarima.box(0, 17.5, 0.6, 16, 7, 1.2, 1.5)
    tarima.save(MALLAS / w / 'tarima.obj')

    ground = Link('piso')
    ground.visual(mesh(w, 'pasto'), tex('pasto'))
    ground.visual(mesh(w, 'adoquin'), tex('adoquin'))
    ground.visual(mesh(w, 'calle'), tex('calle'))
    ground.visual(mesh(w, 'banqueta'), tex('banqueta'))
    ground.collision(box(260, 260, 1), pose(0, 0, -0.5))
    helipad(ground, 0, 0, ring=(1.0, 0.75, 0.2), glow=0.6)

    # --- La estrella pintada en el piso y las 5 balizas -----------------------
    star = Link('estrella')
    gold = color((1.0, 0.72, 0.15), glow=1.0)
    verts = star_vertices(5, R, INNER)
    for (xa, ya), (xb, yb) in zip(verts, verts[1:] + verts[:1]):
        length = math.hypot(xb - xa, yb - ya)
        star.visual(box(length, 0.18, 0.01), gold,
                    pose((xa + xb) / 2, (ya + yb) / 2, 0.03, 0, 0, math.atan2(yb - ya, xb - xa)),
                    shadows=False)

    beacon_colors = [(1.0, 0.75, 0.15), (1.0, 0.2, 0.7), (0.2, 0.85, 1.0), (0.4, 1.0, 0.35), (1.0, 0.45, 0.1)]
    beacons = Link('balizas')
    lights = []
    for k in range(5):
        x, y = verts[2 * k]
        rgb = beacon_colors[k]
        # Base: un reflector en el piso (este sí choca).
        beacons.solid(cyl(0.35, 0.25), color((0.1, 0.1, 0.12), rough=0.3, metal=0.8), pose(x, y, 0.125))
        beacons.visual(cyl(0.25, 0.02), color(rgb, glow=2.0), pose(x, y, 0.26), shadows=False)
        # Haz de luz hasta la altura de vuelo: solo visual, el dron lo atraviesa.
        beacons.visual(cyl(0.12, ALT - 0.3), color(rgb, glow=1.5), pose(x, y, 0.25 + (ALT - 0.3) / 2),
                       transparency=0.8, shadows=False)
        # Aro holográfico a la altura del vértice: el dron tiene que pasar por adentro.
        n = 20
        for i in range(n):
            a = 2 * math.pi * i / n
            beacons.visual(box(0.06, 2 * math.pi * 0.8 / n * 1.05, 0.06), color(rgb, glow=2.0),
                           pose(x + 0.8 * math.cos(a), y + 0.8 * math.sin(a), ALT, 0, 0, a), shadows=False)
        if not concert:
            lights.append(point_light(f'baliza_{k}', x, y, 0.8, rgb, rng=9.0, intensity=2.0))

    # --- Escenario al norte ---------------------------------------------------
    stage = Link('escenario')
    stage.visual(mesh(w, 'tarima'), tex('madera'))
    stage.collision(box(16, 7, 1.2), pose(0, 17.5, 0.6))
    metal = color((0.35, 0.35, 0.38), rough=0.4, metal=0.9)
    for sx in (-8.0, 8.0):
        stage.solid(box(0.5, 0.5, 9), metal, pose(sx, 20.6, 4.5))
    stage.solid(box(16.5, 0.5, 0.5), metal, pose(0, 20.6, 9.0))
    stage.solid(box(13, 0.3, 5.5), color((0.04, 0.04, 0.08), rough=0.2), pose(0, 20.9, 4.5))
    # La pantalla del fondo enseña la misma estrella.
    for (xa, ya), (xb, yb) in zip(verts, verts[1:] + verts[:1]):
        sa = (xa * 0.42, 4.5 + ya * 0.42)
        sb = (xb * 0.42, 4.5 + yb * 0.42)
        length = math.hypot(sb[0] - sa[0], sb[1] - sa[1])
        stage.visual(box(length, 0.05, 0.1), gold,
                     pose((sa[0] + sb[0]) / 2, 20.73, (sa[1] + sb[1]) / 2, 0, -math.atan2(sb[1] - sa[1], sb[0] - sa[0]), 0),
                     shadows=False)
    black = color((0.05, 0.05, 0.05), rough=0.5)
    for sx in (-10.0, 10.0):
        stage.solid(box(1.4, 1.2, 2.8), black, pose(sx, 15.5, 1.4))
        stage.visual(cyl(0.35, 0.05), color((0.2, 0.2, 0.22), rough=0.3), pose(sx, 14.88, 1.9, math.pi / 2, 0, 0))
        stage.visual(cyl(0.2, 0.05), color((0.2, 0.2, 0.22), rough=0.3), pose(sx, 14.88, 0.9, math.pi / 2, 0, 0))
    stage_colors = [(0.9, 0.2, 1.0), (0.2, 0.6, 1.0), (1.0, 0.4, 0.2)]
    for i, sx in enumerate((-5.0, 0.0, 5.0)):
        stage.visual(box(0.4, 0.4, 0.4), black, pose(sx, 20.2, 8.6))
        lights.append(spot_light(f'reflector_{i}', sx, 20.0, 8.5, (0, -0.6, -1), stage_colors[i],
                                 rng=20, intensity=3.0, angle=0.6))

    # --- Alrededores ----------------------------------------------------------
    extras = []
    lamp_spots = [(-PL + 1, -PL + 1), (PL - 1, -PL + 1), (-PL + 1, PL - 1), (PL - 1, PL - 1),
                  (-PL + 1, 0), (PL - 1, 0), (-9, -PL + 1), (9, -PL + 1)]
    for i, (x, y) in enumerate(lamp_spots):
        extras.append(lamp_post(f'poste_{i}', x, y, (x * 0.6, y * 0.6), rng=24, intensity=2.5))
    for i, (x, y, yaw) in enumerate([(-4, -PL + 2, math.pi / 2), (4, -PL + 2, math.pi / 2),
                                      (-PL + 2, -6, 0), (-PL + 2, 6, 0), (PL - 2, -6, 0), (PL - 2, 6, 0)]):
        extras.append(include('bench_long', f'banca_{i}', x, y, 0.0, yaw))
    extras.append(include('fountain', 'fuente', -13, -12, 0.0))
    extras.append(point_light('fuente_luz', -13, -12, 1.0, (0.4, 0.7, 1.0), rng=6, intensity=1.5))
    # Jardineras con árbol (las de los lados quedan lejos del vuelo: radio 5 m).
    planters = Link('jardineras')
    for i, (x, y) in enumerate([(13, -12), (-14, 9), (14, 9)]):
        planters.solid(cyl(1.7, 0.5), tex('concreto_claro'), pose(x, y, 0.25))
        planters.visual(cyl(1.55, 0.02), tex('tierra_pasto'), pose(x, y, 0.51))
        extras.append(include('oak_tree' if i != 1 else 'pine_tree', f'arbol_plaza_{i}', x, y, 0.5, i * 1.3))
    extras.append(static_model('jardineras', [planters]))
    extras.append(include('apartment', 'edificio_1', -55, 15, 0.0, -math.pi / 2))
    extras.append(include('apartment', 'edificio_3', 50, -60, 0.0, math.pi))
    extras.append(include('house_2', 'casa_1', -30, -55, 0.0, math.pi))
    extras.append(include('house_3', 'casa_2', 0, -55, 0.0, math.pi))
    extras.append(include('hatchback_red', 'coche_1', -18, -34.2, 0.0))
    extras.append(include('suv', 'coche_2', 8, -37.8, 0.0, math.pi))
    extras.append(include('pickup', 'coche_3', 30, -34.2, 0.0))
    for i, x in enumerate((-40, -10, 20, 50)):
        extras.append(lamp_post(f'poste_calle_{i}', x, -30.3, (x, -36), intensity=1.2))
    extras.append(trees(1, 34, (-75, -28, 75, 75), [
        rect(-PL - 4, -PL - 4, PL + 4, PL + 4), rect(-80, -45, 80, -27),
        rect(-68, 2, -42, 28), rect(42, 15, 68, 41)]))

    body = '\n'.join([
        sun((0.9, 0.25, -0.35), (1.0, 0.55, 0.32), 0.9),
        '<light type="directional" name="cielo"><cast_shadows>false</cast_shadows><pose>0 0 50 0 0 0</pose>'
        '<diffuse>0.12 0.14 0.28 1</diffuse><specular>0 0 0 1</specular><direction>-0.2 -0.3 -1</direction></light>',
        static_model('plaza', [ground, stage] + ([fence_link(), ] if concert else [star, beacons])),
        *lights, *extras])
    (WORLDS / w).mkdir(parents=True, exist_ok=True)
    if concert:
        f = FENCE
        text = ('Plaza del concierto (misión 4: geocerca).\n'
                f'     La caja roja es la zona permitida: Gazebo x {f["x0"]}..{f["x1"]}, y {f["y0"]}..{f["y1"]},\n'
                f'     hasta {f["top"]} m de alto; en NED desde el despegue: norte {f["y0"]}..{f["y1"]},\n'
                f'     este {f["x0"]}..{f["x1"]}. La línea naranja del piso está 1 m afuera: el\n'
                '     guardián tiene que detener al dron antes de cruzarla.')
        spawn = 'Sobre la H, dentro de la geocerca.'
    else:
        text = ('Plaza de noche para el show de drones (misión 1: estrella).\n'
                f'     La estrella pintada mide {R} m de radio y las balizas marcan sus 5 picos;\n'
                f'     sus aros están a {ALT} m, la altura de vuelo por defecto.')
        spawn = 'Al centro de la estrella, sobre la H.'
    (WORLDS / w / f'{w}.sdf').write_text(world_sdf(w, text, SCENE_DUSK, body))
    spawn_yaml(w, spawn)


def fence_link():
    """La geocerca: paredes rojas translúcidas (solo visuales: el dron las
    atraviesa, por eso hace falta el guardián), aristas que brillan y en el
    piso la orilla (roja) y el límite de tolerancia 1 m afuera (naranja)."""
    f = FENCE
    x0, x1, y0, y1, top = f['x0'], f['x1'], f['y0'], f['y1'], f['top']
    cx, cy, sx, sy = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    link = Link('geocerca')
    wall = color((0.9, 0.05, 0.05), glow=0.2)
    for (px, py, wx, wy) in ((cx, y0, sx, 0.02), (cx, y1, sx, 0.02), (x0, cy, 0.02, sy), (x1, cy, 0.02, sy)):
        link.visual(box(wx, wy, top), wall, pose(px, py, top / 2), transparency=0.93, shadows=False)
    link.visual(box(sx, sy, 0.02), wall, pose(cx, cy, top), transparency=0.95, shadows=False)
    edge = color((1.0, 0.15, 0.1), glow=1.0)
    for x in (x0, x1):
        for y in (y0, y1):
            link.visual(box(0.08, 0.08, top), edge, pose(x, y, top / 2), shadows=False)
    for z in (0.04, top):
        link.visual(box(sx, 0.08, 0.02 if z < 1 else 0.08), edge, pose(cx, y0, z), shadows=False)
        link.visual(box(sx, 0.08, 0.02 if z < 1 else 0.08), edge, pose(cx, y1, z), shadows=False)
        link.visual(box(0.08, sy, 0.02 if z < 1 else 0.08), edge, pose(x0, cy, z), shadows=False)
        link.visual(box(0.08, sy, 0.02 if z < 1 else 0.08), edge, pose(x1, cy, z), shadows=False)
    # Límite de tolerancia: 1 m afuera, en tramos (línea punteada).
    warn = color((1.0, 0.55, 0.05), glow=0.8)
    for (ax, ay, bx, by) in ((x0 - 1, y0 - 1, x1 + 1, y0 - 1), (x0 - 1, y1 + 1, x1 + 1, y1 + 1),
                             (x0 - 1, y0 - 1, x0 - 1, y1 + 1), (x1 + 1, y0 - 1, x1 + 1, y1 + 1)):
        length = math.hypot(bx - ax, by - ay)
        n = int(length / 1.0)
        for i in range(0, n, 2):
            t0 = (i + 0.5) / n
            link.visual(box(length / n, 0.12, 0.01), warn,
                        pose(ax + (bx - ax) * t0, ay + (by - ay) * t0, 0.04, 0, 0, math.atan2(by - ay, bx - ax)),
                        shadows=False)
    return link


# =============================================================================
# Mundo 2: puente_rio (misión 2, inspección del pilar)
# =============================================================================

def build_puente_rio():
    w = 'puente_rio'
    Y0, Y1 = -250, 250
    PILLAR = (6.5, 8.0)       # el pilar a inspeccionar (ENU) -> NED norte 8, este 6.5
    PR = 0.8                   # radio del pilar
    DECK_Z = 7.0               # parte de abajo del tablero
    # Perfil del valle (x, z): meseta, ladera, llanura, orilla, cauce y de vuelta.
    terrain = [
        ('pasto', [(-250, 8), (-18, 8)], 6.0),
        ('tierra_pasto', [(-18, 8), (-9, 0)], 4.0),
        ('pasto', [(-9, 0), (2, 0)], 4.0),
        ('roca', [(2, 0), (5, -3)], 3.0),
        ('lecho_rio', [(5, -3), (23, -3)], 4.0),
        ('roca', [(23, -3), (26, 0)], 3.0),
        ('pasto', [(26, 0), (33, 0)], 4.0),
        ('tierra_pasto', [(33, 0), (42, 8)], 4.0),
        ('pasto', [(42, 8), (270, 8)], 6.0),
    ]
    ground = Link('valle')
    meshes = {}
    for tex_name, prof, tile in terrain:
        meshes.setdefault((tex_name, tile), Obj()).profile(prof, Y0, Y1, tile)
        (xa, za), (xb, zb) = prof
        # Colisión: una caja de 1 m de grueso con su cara de arriba sobre el tramo.
        length = math.hypot(xb - xa, zb - za)
        ang = math.atan2(zb - za, xb - xa)
        nx, nz = -math.sin(ang), math.cos(ang)
        cx, cz = (xa + xb) / 2 - 0.5 * nx, (za + zb) / 2 - 0.5 * nz
        ground.collision(box(length + 0.4, Y1 - Y0, 1.0), pose(cx, (Y0 + Y1) / 2, cz, 0, -ang, 0))
    for i, ((tex_name, tile), obj) in enumerate(meshes.items()):
        obj.save(MALLAS / w / f'terreno_{i}_{tex_name}.obj')
        ground.visual(mesh(w, f'terreno_{i}_{tex_name}'), tex(tex_name))
    ground.visual(box(24, Y1 - Y0, 0.02), color((0.10, 0.30, 0.36), rough=0.05, metal=0.2),
                  pose(14, 0, -1.0), transparency=0.3, shadows=False)
    helipad(ground, 0, 0)

    # --- Puente -----------------------------------------------------------------
    bridge = Link('puente')
    concreto = Obj()
    concreto.box(12, 8, DECK_Z + 0.5, 60, 8, 1.0, 2.0, bottom=True)    # tablero x -18..42
    for x in (-5.5, 6.5, 18.5, 30.5):                                  # cabezales de las pilas
        concreto.box(x, 8, DECK_Z - 0.3, 2.2, 7.4, 0.6, 2.0, bottom=True)
    concreto.box(12, 4.12, DECK_Z + 1.6, 60, 0.25, 1.0, 1.0)          # parapetos
    concreto.box(12, 11.88, DECK_Z + 1.6, 60, 0.25, 1.0, 1.0)
    concreto.save(MALLAS / w / 'puente.obj')
    pilas = Obj()
    for x in (-5.5, 6.5, 18.5, 30.5):
        z0 = 0.0 if x in (-5.5, 30.5) else -3.0
        pilas.cylinder(x, 8, z0, DECK_Z - 0.6, PR, 2.0, top=False)
    pilas.save(MALLAS / w / 'pilas.obj')
    road = Obj()
    road.road(-250, 5, 270, 11, DECK_Z + 1.01, along_x=True)
    road.save(MALLAS / w / 'calle.obj')
    walk = Obj()
    walk.box(10, 4.6, DECK_Z + 1.07, 520, 0.95, 0.14, 2.0)
    walk.box(10, 11.4, DECK_Z + 1.07, 520, 0.95, 0.14, 2.0)
    walk.save(MALLAS / w / 'banqueta.obj')
    bridge.visual(mesh(w, 'puente'), tex('concreto_claro'))
    bridge.visual(mesh(w, 'pilas'), tex('concreto_claro', '0.85 0.85 0.85 1'))
    bridge.visual(mesh(w, 'calle'), tex('calle'))
    bridge.visual(mesh(w, 'banqueta'), tex('banqueta'))
    bridge.collision(box(60, 8, 1.0), pose(12, 8, DECK_Z + 0.5))
    for x in (-5.5, 6.5, 18.5, 30.5):
        z0 = 0.0 if x in (-5.5, 30.5) else -3.0
        bridge.collision(cyl(PR, DECK_Z - 0.6 - z0), pose(x, 8, (z0 + DECK_Z - 0.6) / 2))
        bridge.collision(box(2.2, 7.4, 0.6), pose(x, 8, DECK_Z - 0.3))
    for y in (4.12, 11.88):
        bridge.collision(box(60, 0.25, 1.0), pose(12, y, DECK_Z + 1.6))

    # --- Lo que se inspecciona en el pilar ------------------------------------
    # Bandas a 1, 2, 3 y 4 m sobre la orilla (de donde despega el dron): la
    # hélice de la misión sube justo entre la primera y la última.
    marks = Link('marcas_inspeccion')
    px, py = PILLAR
    band_colors = [(1.0, 0.85, 0.1), (1.0, 0.55, 0.1), (1.0, 0.85, 0.1), (1.0, 0.55, 0.1)]
    for i, z in enumerate((1.0, 2.0, 3.0, 4.0)):
        marks.visual(cyl(PR + 0.015, 0.08), color(band_colors[i], glow=0.4), pose(px, py, z), shadows=False)
    # Grietas: rayas oscuras sobre la superficie, cada una mirando a otro lado.
    rnd = random.Random(7)
    for i in range(9):
        a = rnd.uniform(0, 2 * math.pi)
        z = rnd.uniform(0.6, 4.4)
        marks.visual(box(0.03, 0.02, rnd.uniform(0.3, 0.8)), color((0.08, 0.08, 0.08)),
                     pose(px + (PR + 0.005) * math.cos(a), py + (PR + 0.005) * math.sin(a), z,
                          rnd.uniform(-0.5, 0.5), 0, a), shadows=False)
    # Placas de inspección (amarillo/negro) en las 4 direcciones a 2.5 m.
    for i in range(4):
        a = math.pi / 2 * i
        cx, cy = px + (PR + 0.02) * math.cos(a), py + (PR + 0.02) * math.sin(a)
        marks.visual(box(0.02, 0.4, 0.4), color((1.0, 0.8, 0.0), glow=0.2), pose(cx, cy, 2.5, 0, 0, a))
        marks.visual(box(0.025, 0.2, 0.2), color((0.05, 0.05, 0.05)), pose(cx, cy, 2.5, 0, 0, a))

    # --- Alrededores ----------------------------------------------------------
    extras = []
    for i, x in enumerate((-30, -12, 6, 24, 42, 60)):
        y = 4.6 if i % 2 == 0 else 11.4
        extras.append(lamp_post(f'poste_{i}', x, y, (x, 8), light=False, z=DECK_Z + 1.14))
    extras.append(include('hatchback_red', 'coche_1', -2, 6.5, DECK_Z + 1.02))
    extras.append(include('suv', 'coche_2', 15, 9.5, DECK_Z + 1.02, math.pi))
    extras.append(include('pickup', 'coche_3', 34, 6.5, DECK_Z + 1.02))
    extras.append(include('truck_delivery', 'camioneta_inspeccion', -5, -7, 0.0, math.pi / 2))
    rnd = random.Random(3)
    for i in range(14):
        side = i % 2
        x = rnd.uniform(1.5, 4.2) if side == 0 else rnd.uniform(23.5, 26.5)
        y = rnd.uniform(-80, 80)
        if abs(y - 8) < 6:
            continue
        z = -3 * (x - 2) / 3 if side == 0 else -3 * (26 - x) / 3
        extras.append(include('falling_rock_1', f'roca_{i}', x, y, z - 0.3, rnd.uniform(0, 6.28)))
    keep = [rect(-25, -1, 50, 17), rect(-14, -14, 4, 6), rect(1, -200, 27, 200)]
    extras.append(trees(11, 10, (-9, -90, 1, 90), keep, 7.0))
    extras.append(trees(12, 8, (27, -90, 33, 90), keep, 7.0))
    extras.append(trees(13, 30, (-80, -100, -20, 100), keep, 8.0, 0.6))
    extras.append(trees(14, 30, (44, -100, 110, 100), keep, 8.0, 0.6))
    # Los árboles de las mesetas van a 8 m de alto.
    def lift(xml, dz):
        def fix(m):
            vals = [float(v) for v in m.group(1).split()]
            return '<pose>' + ' '.join(f'{v:.3f}' for v in vals[:2] + [vals[2] + dz] + vals[3:]) + '</pose>'
        return re.sub(r'<pose>([^<]+)</pose>', fix, xml)
    extras[-2] = lift(extras[-2], 8.0)
    extras[-1] = lift(extras[-1], 8.0)

    body = '\n'.join([
        sun((-0.45, 0.5, -0.75), (1.0, 0.96, 0.88), 1.0),
        static_model('escena', [ground, bridge, marks]),
        *extras])
    (WORLDS / w).mkdir(parents=True, exist_ok=True)
    (WORLDS / w / f'{w}.sdf').write_text(world_sdf(
        w, 'Río con un puente de concreto (misión 2: inspeccionar un pilar).\n'
        f'     El pilar con bandas de colores está en Gazebo ({px}, {py}), o sea\n'
        f'     NED norte {py} m, este {px} m desde donde despega el dron. Mide {PR} m\n'
        '     de radio y las bandas están a 1, 2, 3 y 4 m sobre la orilla.', SCENE_DAY, body))
    spawn_yaml(w, 'En la orilla oeste, 4 m al sur del puente.')


# =============================================================================
# Mundo 3: barrio_entrega (misión 3, entrega a domicilio)
# =============================================================================

def build_barrio_entrega():
    w = 'barrio_entrega'
    TARGET = (16.0, 36.0)      # plataforma de entrega en el patio (ENU)

    pasto = Obj()
    pasto.plane(-140, -140, 140, 140, 0.0, 4.0)
    pasto.save(MALLAS / w / 'pasto.obj')
    calles = Obj()
    calles.road(-140, 6, 140, 14, 0.012, along_x=True)              # calle principal (este-oeste)
    calles.road(-44, -140, -36, 4, 0.012, along_x=False)            # calle transversal, partida
    calles.road(-44, 16, -36, 140, 0.012, along_x=False)
    calles.save(MALLAS / w / 'calles.obj')
    crucero = Obj()
    crucero.plane(-44, 4, -36, 16, 0.013, 3.0)
    crucero.save(MALLAS / w / 'crucero.obj')
    banq = Obj()
    for x0, x1 in ((-140, -46), (-34, 140)):
        banq.box((x0 + x1) / 2, 5, 0.06, x1 - x0, 2, 0.12, 2.0)
        banq.box((x0 + x1) / 2, 15, 0.06, x1 - x0, 2, 0.12, 2.0)
    for y0, y1 in ((-140, 4), (16, 140)):
        banq.box(-45, (y0 + y1) / 2, 0.06, 2, y1 - y0, 0.12, 2.0)
        banq.box(-35, (y0 + y1) / 2, 0.06, 2, y1 - y0, 0.12, 2.0)
    banq.save(MALLAS / w / 'banquetas.obj')
    lote = Obj()
    lote.plane(-15, -6, 15, 4, 0.015, 3.0)
    lote.save(MALLAS / w / 'estacionamiento.obj')
    farmacia = Obj()
    farmacia.box(0, -11, 2.75, 18, 8, 5.5, 2.0)
    farmacia.save(MALLAS / w / 'farmacia.obj')
    techo = Obj()
    techo.box(0, -11, 5.65, 18.6, 8.6, 0.3, 2.0)
    techo.save(MALLAS / w / 'farmacia_techo.obj')

    ground = Link('piso')
    ground.visual(mesh(w, 'pasto'), tex('pasto'))
    ground.visual(mesh(w, 'calles'), tex('calle'))
    ground.visual(mesh(w, 'crucero'), tex('asfalto', '0.45 0.45 0.45 1'))
    ground.visual(mesh(w, 'banquetas'), tex('banqueta'))
    ground.visual(mesh(w, 'estacionamiento'), tex('asfalto', '0.5 0.5 0.5 1'))
    ground.collision(box(280, 280, 1), pose(0, 0, -0.5))
    helipad(ground, 0, 0, ring=(0.1, 0.8, 0.35))
    white = color((0.92, 0.92, 0.92))
    for x in (-13, -10, -7, 7, 10, 13):           # cajones del estacionamiento
        ground.visual(box(0.12, 4.5, 0.01), white, pose(x, -3.6, 0.03), shadows=False)

    # --- Farmacia (de donde sale el pedido) -----------------------------------
    shop = Link('farmacia')
    shop.visual(mesh(w, 'farmacia'), tex('ladrillo'))
    shop.visual(mesh(w, 'farmacia_techo'), tex('concreto_claro'))
    shop.collision(box(18.6, 8.6, 5.8), pose(0, -11, 2.9))
    glass = color((0.25, 0.4, 0.5), rough=0.05, metal=0.6)
    shop.visual(box(2.4, 0.06, 2.5), glass, pose(0, -6.98, 1.25))
    for x in (-5.5, 5.5):
        shop.visual(box(4.0, 0.06, 2.0), glass, pose(x, -6.98, 1.8))
    green = color((0.1, 0.9, 0.35), glow=1.2)
    shop.visual(box(1.5, 0.1, 0.5), green, pose(0, -6.92, 4.25), shadows=False)
    shop.visual(box(0.5, 0.1, 1.5), green, pose(0, -6.92, 4.25), shadows=False)
    shop.visual(box(16, 0.08, 0.7), color((0.95, 0.95, 0.95)), pose(0, -6.94, 3.0))
    shop.visual(box(16.2, 0.1, 0.12), color((0.1, 0.6, 0.3)), pose(0, -6.93, 3.4))

    # --- Patio de la entrega ----------------------------------------------------
    yard = Link('patio_entrega')
    tx, ty = TARGET
    yard.visual(box(2.8, 2.8, 0.02), white, pose(tx, ty, 0.02), shadows=False)
    yard.visual(box(2.5, 2.5, 0.02), color((0.95, 0.45, 0.08), glow=0.15), pose(tx, ty, 0.03), shadows=False)
    yard.visual(cyl(0.85, 0.01), white, pose(tx, ty, 0.045), shadows=False)
    yard.visual(cyl(0.7, 0.01), color((0.95, 0.45, 0.08)), pose(tx, ty, 0.05), shadows=False)
    yard.visual(box(0.6, 0.6, 0.01), white, pose(tx, ty, 0.06), shadows=False)

    fences = Obj()
    fence_links = Link('cercas')
    for cx in (-24.0, -2.0, 16.0, 40.0):        # patio de cada casa del lado norte
        x0, x1, y0, y1 = cx - 9, cx + 9, 31.0, 46.0
        for (ax, ay, bx, by) in ((x0, y0, x0, y1), (x1, y0, x1, y1), (x0, y1, x1, y1)):
            sx, sy = max(abs(bx - ax), 0.08), max(abs(by - ay), 0.08)
            fences.box((ax + bx) / 2, (ay + by) / 2, 0.75, sx, sy, 1.5, 1.5)
            fence_links.collision(box(sx, sy, 1.5), pose((ax + bx) / 2, (ay + by) / 2, 0.75))
    fences.save(MALLAS / w / 'cercas.obj')
    fence_links.visual(mesh(w, 'cercas'), tex('madera'))

    # --- Casas, coches y lo demás ---------------------------------------------
    # El frente de las casas de Fuel mira a -y: con yaw 0 dan a la calle del sur.
    extras = [
        include('house_1', 'casa_1', -24, 25.5),
        include('house_2', 'casa_2', -2, 24.5),
        include('house_3', 'casa_destino', 16, 25.0),
        include('house_1', 'casa_4', 40, 25.5),
        include('house_2', 'casa_5', -62, 24.5),
        include('house_3', 'casa_6', -28, -16, 0.0, math.pi),
        include('house_1', 'casa_7', 32, -18, 0.0, math.pi),
        include('house_2', 'casa_8', 58, -16, 0.0, math.pi),
        include('house_3', 'casa_9', -62, -16, 0.0, math.pi),
        include('house_1', 'casa_10', -20, 62),
        include('house_3', 'casa_11', 10, 62),
        include('house_2', 'casa_12', 40, 62),
        include('water_tower', 'tinaco', -80, 80),
        include('apartment', 'edificio', 95, 40, 0.0, math.pi / 2),
        include('hatchback_blue', 'coche_1', -10.5, -3.0, 0.0, math.pi / 2),
        include('suv', 'coche_2', 8.5, -3.0, 0.0, math.pi / 2),
        include('pickup', 'coche_3', 28, 7.3),
        include('hatchback_red', 'coche_4', -20, 12.7, 0.0, math.pi),
        include('truck_delivery', 'camion', -58, 7.6),
        include('suv', 'coche_5', -40, 30, 0.0, -math.pi / 2),
    ]
    for i, x in enumerate((-110, -85, -60, -20, 5, 30, 55, 80, 105)):
        extras.append(lamp_post(f'poste_{i}', x, 4.4, (x, 10), light=False))
    keep = [rect(-140, 2, 140, 18), rect(-48, -140, -32, 140), rect(-17, -17, 17, 6),
            rect(-34, 16, 50, 48), rect(-72, 16, -52, 34), rect(-75, -25, 70, -7),
            rect(-30, 52, 52, 72), rect(-90, 70, -70, 90), rect(80, 25, 110, 55), rect(75, -75, 105, -45)]
    extras.append(trees(21, 45, (-130, -130, 130, 130), keep, 9.0))
    pts = [(x, 17.2) for x in (-120, -95, -70, -55, -30, -12, 8, 26, 48, 70, 95, 120)]
    pts += [(x, 2.8) for x in (-120, -97, -72, -50, -25, 20, 45, 70, 95, 120)]
    extras += [include('oak_tree', f'arbol_banqueta_{i}', x, y, 0.0, i) for i, (x, y) in enumerate(pts)]

    north, east = ty, tx
    dist = math.hypot(north, east)
    body = '\n'.join([
        sun((-0.5, 0.35, -0.8), (1.0, 0.96, 0.88), 1.0),
        static_model('escena', [ground, shop, yard, fence_links]),
        *extras])
    (WORLDS / w).mkdir(parents=True, exist_ok=True)
    lat, lon = ned_to_gps(north, east)
    (WORLDS / w / f'{w}.sdf').write_text(world_sdf(
        w, 'Barrio con una farmacia y un patio de entrega (misión 3).\n'
        f'     El dron sale de la H verde de la farmacia. La plataforma naranja está\n'
        f'     en el patio de la casa de enfrente: NED norte {north} m, este {east} m\n'
        f'     ({dist:.1f} m), o sea lat {lat:.7f}, lon {lon:.7f}.', SCENE_DAY, body))
    spawn_yaml(w, 'En la H verde del estacionamiento de la farmacia.')


def ned_to_gps(north, east):
    """Metros a lat/lon con los radios del elipsoide WGS84 (el que usa Gazebo)."""
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    s = math.sin(math.radians(LAT0))
    meridian = a * (1 - e2) / (1 - e2 * s * s) ** 1.5
    normal = a / math.sqrt(1 - e2 * s * s)
    lat = LAT0 + math.degrees(north / meridian)
    lon = LON0 + math.degrees(east / (normal * math.cos(math.radians(LAT0))))
    return lat, lon


# =============================================================================
# Mundo 4: cueva (misión 5, cruzar una cueva con el lidar)
# =============================================================================

def catmull_rom(points, step):
    """Curva suave que pasa por `points`, muestreada cada ~`step` metros."""
    pts = [points[0]] + points + [points[-1]]
    out = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
        n = max(2, int(math.dist(p1, p2) / step))
        for k in range(n):
            t = k / n
            t2, t3 = t * t, t * t * t
            out.append(tuple(0.5 * ((2 * p1[j]) + (-p0[j] + p2[j]) * t + (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * t2
                                    + (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * t3) for j in range(2)))
    out.append(points[-1])
    return out


def bump(s, center, length):
    """0 lejos de `center`, 1 en el centro, con transición suave (coseno)."""
    d = abs(s - center) / (length / 2)
    return 0.5 * (1 + math.cos(math.pi * d)) if d < 1 else 0.0


def rock_noise(a, b, seed=0.0):
    """Ruido suave en [0, 1] para que la roca no se vea lisa."""
    v = (math.sin(0.9 * a + 1.7 * b + seed) * 0.5 + math.sin(2.3 * a - 0.8 * b + 2 * seed) * 0.3
         + math.sin(4.1 * a + 3.3 * b + 3 * seed) * 0.2)
    return 0.5 + 0.5 * v


def build_cueva():
    w = 'cueva'
    H = 4.0                    # alto del túnel
    Y_IN, Y_OUT = 10.0, 76.0   # caras sur y norte de la montaña (Gazebo y)
    TOP = 10.0                 # altura de la montaña
    MX = 70.0                  # la loma llega al suelo en x = -MX y x = MX
    # El camino del túnel (Gazebo x, y). Se recorta entre Y_IN y Y_OUT.
    path = catmull_rom([(0, 0), (0, 10), (0, 16), (2, 23), (6, 30), (8, 37), (7, 44), (3, 50),
                        (-1, 56), (-3, 63), (-2, 70), (-1, 76), (-1, 86)], 0.5)
    # El túnel sale 1 m de cada cara para que la boca no deje ver huecos.
    path = [p for p in path if Y_IN - 1.0 <= p[1] <= Y_OUT + 1.0]
    # Distancia recorrida en cada punto y dirección hacia la izquierda.
    s_at, normals = [0.0], []
    for i in range(1, len(path)):
        s_at.append(s_at[-1] + math.dist(path[i - 1], path[i]))
    for i in range(len(path)):
        a, b = path[max(0, i - 1)], path[min(len(path) - 1, i + 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        n = math.hypot(tx, ty)
        normals.append((-ty / n, tx / n))
    length = s_at[-1]

    NARROW_1, NARROW_2, PILLAR_S = 18.0, 50.0, 34.0

    def width(s):
        # 5 m normal, dos angosturas de 3 m y más ancho junto a la columna.
        return 5.0 - 2.0 * bump(s, NARROW_1, 10.0) - 1.8 * bump(s, NARROW_2, 9.0) + 1.0 * bump(s, PILLAR_S, 12.0)

    walls, ceiling, floor_ = Obj(), Obj(), Obj()
    cave = Link('cueva')
    zs = [H * k / 8 for k in range(9)]
    for side, seed in ((1, 0.0), (-1, 5.0)):      # 1 = pared izquierda, -1 = derecha
        def wall_point(i, z):
            c, n = path[i], normals[i]
            off = width(s_at[i]) / 2 - 0.3 * rock_noise(s_at[i], z, seed)   # la roca sale hacia adentro
            return (c[0] + side * n[0] * off, c[1] + side * n[1] * off, z)
        for i in range(len(path) - 1):
            for j in range(len(zs) - 1):
                pts = [wall_point(i, zs[j]), wall_point(i + 1, zs[j]), wall_point(i + 1, zs[j + 1]), wall_point(i, zs[j + 1])]
                uvs = [(s_at[i] / 3, zs[j] / 3), (s_at[i + 1] / 3, zs[j] / 3), (s_at[i + 1] / 3, zs[j + 1] / 3), (s_at[i] / 3, zs[j + 1] / 3)]
                walls.quad_facing(pts, uvs, (path[i][0], path[i][1], zs[j]))
        # Colisión: cajas sobre la pared "nominal" (sin el ruido), cada ~1 m.
        for i in range(0, len(path) - 2, 2):
            a, b = path[i], path[i + 2]
            na, nb = normals[i], normals[i + 2]
            wa, wb = width(s_at[i]) / 2 + 0.25, width(s_at[i + 2]) / 2 + 0.25
            pa = (a[0] + side * na[0] * wa, a[1] + side * na[1] * wa)
            pb = (b[0] + side * nb[0] * wb, b[1] + side * nb[1] * wb)
            seg = math.dist(pa, pb)
            cave.collision(box(seg + 0.3, 0.5, H), pose((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2, H / 2, 0, 0,
                                                      math.atan2(pb[1] - pa[1], pb[0] - pa[0])))
    # Techo y piso: franjas de pared a pared.
    for i in range(len(path) - 1):
        for k in range(4):
            def across(idx, frac, z):
                c, n = path[idx], normals[idx]
                half = width(s_at[idx]) / 2
                d = half - 2 * half * frac
                return (c[0] + n[0] * d, c[1] + n[1] * d, z)
            f0, f1 = k / 4, (k + 1) / 4
            zc = [H - 0.35 * rock_noise(s_at[idx], f, 9.0) for idx, f in ((i, f0), (i + 1, f0), (i + 1, f1), (i, f1))]
            pts = [across(i, f0, zc[0]), across(i + 1, f0, zc[1]), across(i + 1, f1, zc[2]), across(i, f1, zc[3])]
            ceiling.quad_facing(pts, [(p[0] / 3, p[1] / 3) for p in pts], (path[i][0], path[i][1], 0.0))
            pts = [across(i, f0, 0.02), across(i + 1, f0, 0.02), across(i + 1, f1, 0.02), across(i, f1, 0.02)]
            floor_.quad_facing(pts, [(p[0] / 2, p[1] / 2) for p in pts], (path[i][0], path[i][1], 5.0))
        if i % 4 == 0:
            c = path[i]
            cave.collision(box(width(s_at[i]) + 1.5, 2.2, 0.5), pose(c[0], c[1], H + 0.25, 0, 0,
                                                                     math.atan2(normals[i][1], normals[i][0])))
    walls.save(MALLAS / w / 'paredes.obj')
    ceiling.save(MALLAS / w / 'techo.obj')
    floor_.save(MALLAS / w / 'piso_cueva.obj')
    cave.visual(mesh(w, 'paredes'), tex('roca', '0.55 0.52 0.48 1'))
    cave.visual(mesh(w, 'techo'), tex('roca', '0.45 0.43 0.4 1'))
    cave.visual(mesh(w, 'piso_cueva'), tex('grava', '0.5 0.5 0.5 1'))

    # --- La columna de roca: de piso a techo, cargada a la derecha ------------
    ip = min(range(len(path)), key=lambda i: abs(s_at[i] - PILLAR_S))
    c, n = path[ip], normals[ip]
    px, py = c[0] - n[0] * 1.4, c[1] - n[1] * 1.4
    column = Obj()
    sides = 24
    for j in range(8):
        z0, z1 = H * j / 8, H * (j + 1) / 8
        for k in range(sides):
            ring = []
            for kk, z in ((k, z0), (k + 1, z0), (k + 1, z1), (k, z1)):
                a = 2 * math.pi * kk / sides
                r = 0.7 + 0.18 * rock_noise(a * 2, z, 3.0) + 0.15 * bump(z, 0.0, 2.0) + 0.15 * bump(z, H, 2.0)
                ring.append((px + r * math.cos(a), py + r * math.sin(a), z))
            column.quad_facing(ring, [(k / 4, z0 / 2), ((k + 1) / 4, z0 / 2), ((k + 1) / 4, z1 / 2), (k / 4, z1 / 2)],
                               (px, py, (z0 + z1) / 2), outward=True)
    column.save(MALLAS / w / 'columna.obj')
    cave.visual(mesh(w, 'columna'), tex('roca', '0.8 0.75 0.7 1'))
    cave.collision(cyl(0.85, H), pose(px, py, H / 2))

    # --- Lámparas en las paredes cada 8 m -------------------------------------
    lights = []
    lamp_mat = color((1.0, 0.75, 0.4), glow=1.0)
    for k, s in enumerate(range(4, int(length) - 2, 8)):
        i = min(range(len(path)), key=lambda q: abs(s_at[q] - s))
        c, n = path[i], normals[i]
        side = 1 if k % 2 == 0 else -1
        d = width(s_at[i]) / 2 - 0.35
        lx, ly = c[0] + side * n[0] * d, c[1] + side * n[1] * d
        cave.visual(box(0.3, 0.18, 0.14), lamp_mat, pose(lx, ly, 3.0, 0, 0, math.atan2(n[1], n[0])), shadows=False)
        lights.append(point_light(f'lampara_{k}', c[0] + side * n[0] * (d - 0.4), c[1] + side * n[1] * (d - 0.4),
                                  2.8, (1.0, 0.72, 0.4), rng=9.0, intensity=1.6))

    # --- La montaña por fuera: una loma que el túnel atraviesa ---------------
    # Alta (TOP) en |x| < 30 y bajando suave hasta el suelo en |x| = MX; en
    # las caras sur y norte, acantilados con la boca del túnel.
    mount = Obj()
    faces = Obj()

    def ridge(x):
        d = (abs(x) - 30.0) / (MX - 30.0)
        return 1.0 if d <= 0 else (0.5 * (1 + math.cos(math.pi * d)) if d < 1 else 0.0)

    def top_h(x, y):
        return ridge(x) * (TOP + 1.5 * (rock_noise(x / 7, y / 7, 1.0) - 0.5))

    step = 2.0
    xs = [-MX + step * k for k in range(int(2 * MX / step) + 1)]
    ys = [Y_IN + (Y_OUT - Y_IN) * k / 33 for k in range(34)]
    for a in range(len(xs) - 1):
        for b in range(len(ys) - 1):
            pts = [(xs[a], ys[b], top_h(xs[a], ys[b])), (xs[a + 1], ys[b], top_h(xs[a + 1], ys[b])),
                   (xs[a + 1], ys[b + 1], top_h(xs[a + 1], ys[b + 1])), (xs[a], ys[b + 1], top_h(xs[a], ys[b + 1]))]
            mount.quad_facing(pts, [(p[0] / 6, p[1] / 6) for p in pts], (pts[0][0], pts[0][1], 100.0))

    def cliff(y_face, out_dir, mouth_x):
        """Cara de roca en y = y_face con la boca del túnel en x = mouth_x."""
        # La boca es un poco más chica que el túnel (2.25 m de medio ancho y
        # 3.65 m de alto como mínimo, por la roca de adentro): la cara la
        # enmarca sin dejar huecos. Las celdas se alinean con sus orillas.
        half, mouth_h, grid = 2.0, 3.5, 1.0
        k0 = int((-MX - mouth_x) // grid)
        k1 = int((MX - mouth_x) // grid) + 1
        for k in range(k0, k1):
            x0, x1 = mouth_x + grid * k, mouth_x + grid * (k + 1)
            zt0, zt1 = top_h(x0, y_face), top_h(x1, y_face)
            if max(zt0, zt1) < 0.05:
                continue
            in_mouth = x0 >= mouth_x - half - 1e-6 and x1 <= mouth_x + half + 1e-6
            levels = [0.5 * q for q in range(int(min(zt0, zt1) / 0.5) + 1)]
            rows = list(zip(levels, levels[1:])) + [(levels[-1], None)]
            for za, zb in rows:
                if in_mouth and (zb is not None and zb <= mouth_h + 1e-6):
                    continue   # la boca
                zb0 = zt0 if zb is None else zb
                zb1 = zt1 if zb is None else zb
                def p(x, z, top):
                    # el ruido se apaga arriba (para no despegarse de la loma) y junto a la boca
                    fade = (1 - z / max(top, 0.1)) * min(1.0, max(0.0, abs(x - mouth_x) - half) / 2.0)
                    return (x, y_face + out_dir * 0.7 * rock_noise(x, z, 4.0) * fade, z)
                pts = [p(x0, za, zt0), p(x1, za, zt1), p(x1, zb1, zt1), p(x0, zb0, zt0)]
                xm, zm = (x0 + x1) / 2, (za + zb0) / 2
                faces.quad_facing(pts, [(q[0] / 4, q[2] / 4) for q in pts], (xm, y_face + out_dir * 10, zm))
        # Colisiones: a los lados de la boca y arriba de ella (donde la loma es alta).
        y_c = y_face - out_dir * 0.5
        left, right = mouth_x - half, mouth_x + half
        cave.collision(box(left + 35, 1.0, TOP), pose((left - 35) / 2, y_c, TOP / 2))
        cave.collision(box(35 - right, 1.0, TOP), pose((right + 35) / 2, y_c, TOP / 2))
        cave.collision(box(2 * half, 1.0, TOP - H), pose(mouth_x, y_c, (TOP + H) / 2))

    cliff(Y_IN, -1, path[0][0])
    cliff(Y_OUT, 1, path[-1][0])
    mount.save(MALLAS / w / 'montana_arriba.obj')
    faces.save(MALLAS / w / 'montana_caras.obj')
    cave.visual(mesh(w, 'montana_arriba'), tex('tierra_pasto'))
    cave.visual(mesh(w, 'montana_caras'), tex('roca'))

    # --- Afuera ---------------------------------------------------------------
    ground = Link('suelo')
    suelo = Obj()
    suelo.plane(-150, -150, 150, 150, 0.0, 4.0)
    suelo.save(MALLAS / w / 'suelo.obj')
    ground.visual(mesh(w, 'suelo'), tex('tierra_pasto'))
    ground.collision(box(300, 300, 1), pose(0, 0, -0.5))
    helipad(ground, 0, 0, ring=(1.0, 0.85, 0.1))
    ex, ey = path[-1][0], Y_OUT + 8
    ground.visual(box(2.8, 2.8, 0.02), color((0.92, 0.92, 0.92)), pose(ex, ey, 0.02), shadows=False)
    ground.visual(box(2.5, 2.5, 0.02), color((0.95, 0.45, 0.08), glow=0.15), pose(ex, ey, 0.03), shadows=False)
    ground.visual(cyl(0.85, 0.01), color((0.92, 0.92, 0.92)), pose(ex, ey, 0.045), shadows=False)

    extras = [include('truck_delivery', 'camion_rescate', -6, -4, 0.0, math.pi / 2)]
    rnd = random.Random(31)
    for i in range(16):
        side = -1 if i % 2 else 1
        x = side * rnd.uniform(5, 30)
        y = rnd.choice((rnd.uniform(5, 9), rnd.uniform(77, 82)))
        extras.append(include('falling_rock_1', f'roca_{i}', x, y, -0.3, rnd.uniform(0, 6.28)))
    keep = [rect(-MX, Y_IN - 3, MX, Y_OUT + 3), rect(-10, -8, 10, 9), rect(-8, 78, 6, 95)]
    extras.append(trees(41, 40, (-120, -60, 120, 140), keep, 8.0, 0.75))
    # Pinos sobre la loma, a la altura del terreno.
    pts, rnd2 = scatter(42, 45, (-MX + 5, Y_IN + 4, MX - 5, Y_OUT - 4), [], 7.0)
    for i, (x, y) in enumerate(pts):
        extras.append(include('pine_tree', f'pino_loma_{i}', x, y, top_h(x, y) - 0.2, rnd2.uniform(0, 6.28)))
    north, east = ey, ex
    body = '\n'.join([
        sun((-0.4, 0.45, -0.8), (1.0, 0.96, 0.88), 1.2),
        static_model('escena', [ground, cave]),
        *lights, *extras])
    (WORLDS / w).mkdir(parents=True, exist_ok=True)
    (WORLDS / w / f'{w}.sdf').write_text(world_sdf(
        w, 'Cueva que atraviesa una montaña (misión 5: cruzarla con el lidar 2D).\n'
        f'     Correr con el modelo x500_lidar_2d. El túnel mide ~{length:.0f} m, 5 m de ancho\n'
        f'     y {H} m de alto, con dos angosturas de 3 m y una columna de roca a medio\n'
        '     camino que deja un solo paso (por la izquierda). La boca sur está 10 m al\n'
        f'     norte del despegue; la plataforma de salida en NED norte {north:.1f}, este {east:.1f}.',
        SCENE_CAVE, body))
    spawn_yaml(w, 'Frente a la boca sur de la cueva.')


SCENE_CAVE = ('<scene><ambient>0.18 0.18 0.2 1</ambient><background>0.62 0.75 0.92 1</background>'
              '<sky></sky><shadows>true</shadows><grid>false</grid></scene>')


# =============================================================================
# Piezas del Nivel 3: ArUco, bodegas, contenedores, cerca
# =============================================================================

def aruco_plate(link, marker_id, x, y, z, size, yaw=0.0):
    """Placa con un ArUco (models/x500_aruco, de tools/make_aruco.py), boca arriba."""
    mat = (f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>'
           f'<pbr><metal><albedo_map>model://x500_aruco/aruco_{marker_id}.png</albedo_map>'
           f'<roughness>1</roughness><metalness>0</metalness></metal></pbr></material>')
    link.visual(box(size, size, 0.01), mat, pose(x, y, z, 0, 0, yaw), shadows=False)


def warehouse(walls, link, x, y, sx, sy, h, door_side='S', roof_rgb=(0.55, 0.57, 0.6)):
    """Bodega: muros (malla con textura, en `walls`), techo de lámina,
    cortinas metálicas en un costado y una franja de color."""
    walls.box(x, y, h / 2, sx, sy, h, 2.0)
    link.collision(box(sx, sy, h), pose(x, y, h / 2))
    link.visual(box(sx + 0.6, sy + 0.6, 0.35), color(roof_rgb, rough=0.35, metal=0.7), pose(x, y, h + 0.15))
    stripe = color((0.1, 0.35, 0.65))
    link.visual(box(sx + 0.04, sy + 0.04, 0.5), stripe, pose(x, y, h - 0.6))
    door = color((0.38, 0.4, 0.42), rough=0.4, metal=0.6)
    n = max(1, int((sx if door_side in 'NS' else sy) / 7))
    for i in range(n):
        f = (i + 0.5) / n - 0.5
        if door_side in 'NS':
            dy = -sy / 2 - 0.03 if door_side == 'S' else sy / 2 + 0.03
            link.visual(box(4.0, 0.06, 4.5), door, pose(x + f * sx, y + dy, 2.25))
        else:
            dx = -sx / 2 - 0.03 if door_side == 'W' else sx / 2 + 0.03
            link.visual(box(0.06, 4.0, 4.5), door, pose(x + dx, y + f * sy, 2.25))


CONTAINER_COLORS = [(0.7, 0.15, 0.1), (0.1, 0.3, 0.6), (0.15, 0.45, 0.2), (0.85, 0.5, 0.1), (0.5, 0.5, 0.52)]


def containers(link, x, y, rows, cols, levels, yaw=0.0, seed=0):
    """Pila de contenedores de 6.1 x 2.44 x 2.6 m."""
    rnd = random.Random(seed)
    c, s = math.cos(yaw), math.sin(yaw)
    for r in range(rows):
        for k in range(cols):
            for lv in range(levels if (r + k) % 2 == 0 else max(1, levels - 1)):
                lx, ly = r * 6.4, k * 2.6
                px, py = x + lx * c - ly * s, y + lx * s + ly * c
                rgb = rnd.choice(CONTAINER_COLORS)
                link.solid(box(6.1, 2.44, 2.6), color(rgb, rough=0.5, metal=0.4), pose(px, py, 1.3 + 2.6 * lv, 0, 0, yaw))
                for e in (-1, 1):   # puertas: un tono más oscuro al frente
                    link.visual(box(0.05, 2.3, 2.45), color(tuple(v * 0.7 for v in rgb), rough=0.5, metal=0.4),
                                pose(px + e * 3.06 * c, py + e * 3.06 * s, 1.3 + 2.6 * lv, 0, 0, yaw))


def fence(link, x0, y0, x1, y1, gate=None, h=2.4):
    """Cerca de malla alrededor del rectángulo; gate = (x_centro, ancho) en el lado sur."""
    post = color((0.45, 0.47, 0.5), rough=0.4, metal=0.8)
    mesh_mat = color((0.6, 0.62, 0.65), rough=0.5, metal=0.6)
    sides = [((x0, y1), (x1, y1)), ((x0, y0), (x0, y1)), ((x1, y0), (x1, y1))]
    if gate:
        gx, gw = gate
        sides += [((x0, y0), (gx - gw / 2, y0)), ((gx + gw / 2, y0), (x1, y0))]
    else:
        sides.append(((x0, y0), (x1, y0)))
    for (ax, ay), (bx, by) in sides:
        length = math.hypot(bx - ax, by - ay)
        ang = math.atan2(by - ay, bx - ax)
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        link.visual(box(length, 0.02, h), mesh_mat, pose(cx, cy, h / 2, 0, 0, ang), transparency=0.6, shadows=False)
        link.collision(box(length, 0.1, h), pose(cx, cy, h / 2, 0, 0, ang))
        link.visual(box(length, 0.05, 0.05), post, pose(cx, cy, h, 0, 0, ang))
        n = max(1, int(length / 3))
        for i in range(n + 1):
            t = i / n
            link.visual(cyl(0.04, h), post, pose(ax + (bx - ax) * t, ay + (by - ay) * t, h / 2))


# =============================================================================
# Mundo 5: zona_bodegas (misión 6, caza del objetivo con la cámara)
# =============================================================================

# La camioneta sospechosa da vueltas en el anillo vial (lo mismo que dicen el
# .sdf y el README): centro, radio del carril y velocidad.
RING_C, RING_R, SUSPECT_SPEED = (0.0, 42.0), 26.5, 3.0


def build_zona_bodegas():
    w = 'zona_bodegas'
    X0, X1, Y0, Y1 = -55.0, 55.0, -12.0, 85.0      # el patio (la cerca)
    cx, cy = RING_C

    pasto = Obj()
    pasto.plane(-150, -150, 150, 180, 0.0, 4.0)
    pasto.save(MALLAS / w / 'pasto.obj')
    patio = Obj()
    patio.plane(X0, Y0, X1, Y1, 0.01, 3.0)
    patio.save(MALLAS / w / 'patio.obj')
    calles = Obj()
    calles.ring_road(cx, cy, 23.0, 30.0, 0.02)
    calles.save(MALLAS / w / 'anillo.obj')
    acceso = Obj()
    acceso.road(-4, -30, 4, cy - 29.5, 0.02, along_x=False)
    acceso.save(MALLAS / w / 'acceso.obj')

    ground = Link('piso')
    ground.visual(mesh(w, 'pasto'), tex('pasto'))
    ground.visual(mesh(w, 'patio'), tex('concreto_claro', '0.8 0.8 0.8 1'))
    ground.visual(mesh(w, 'anillo'), tex('calle'))
    ground.visual(mesh(w, 'acceso'), tex('calle'))
    ground.collision(box(300, 330, 1), pose(0, 15, -0.5))
    helipad(ground, 0, 0, ring=(0.2, 0.6, 1.0))

    # --- Bodegas, contenedores, caseta -----------------------------------------
    yard = Link('patio')
    walls = Obj()
    warehouse(walls, yard, cx, cy, 26, 16, 9, door_side='S')
    warehouse(walls, yard, -40, 62, 22, 14, 8, door_side='E', roof_rgb=(0.6, 0.35, 0.3))
    warehouse(walls, yard, 40, 62, 22, 14, 8, door_side='W')
    warehouse(walls, yard, 42, 18, 16, 22, 7, door_side='W', roof_rgb=(0.35, 0.45, 0.4))
    walls.box(8, -6, 1.5, 4, 3, 3, 1.5)                 # caseta de vigilancia
    yard.collision(box(4, 3, 3), pose(8, -6, 1.5))
    walls.save(MALLAS / w / 'muros.obj')
    yard.visual(mesh(w, 'muros'), tex('concreto_claro', '0.9 0.88 0.82 1'))
    yard.visual(box(4.4, 3.4, 0.2), color((0.2, 0.2, 0.22)), pose(8, -6, 3.1))
    yard.visual(box(0.06, 1.6, 1.0), color((0.3, 0.45, 0.55), rough=0.05, metal=0.6), pose(5.97, -6, 1.8))
    containers(yard, -48, 14, 2, 4, 2, seed=1)
    containers(yard, -30, 78, 3, 2, 3, yaw=0.0, seed=2)
    containers(yard, 22, 78, 3, 2, 2, yaw=0.0, seed=3)
    fence(yard, X0, Y0, X1, Y1, gate=(0.0, 10.0))
    # pluma de la entrada
    yard.visual(box(0.3, 0.3, 1.1), color((0.9, 0.85, 0.1)), pose(-5.4, Y0, 0.55))
    yard.visual(box(9.0, 0.12, 0.12), color((0.95, 0.15, 0.1)), pose(-0.9, Y0, 1.0))

    # --- Coches estacionados con ArUco: los señuelos --------------------------
    extras = []
    decoys = [('hatchback_blue', 3, -38.0, 36.0, 0.0, 1.62), ('pickup', 12, 36.0, 40.0, math.pi / 2, 1.92),
              ('suv', 25, -12.0, 79.0, 0.0, 2.2)]
    for model, marker_id, x, y, yaw, h in decoys:
        extras.append(include(model, f'coche_aruco_{marker_id}', x, y, 0.0, yaw))
        aruco_plate(yard, marker_id, x, y, h, 1.2, yaw)
    extras.append(include('truck_delivery', 'camion_1', -28, 50, 0.0, math.pi / 2))
    extras.append(include('truck_delivery', 'camion_2', 30, 30, 0.0, -math.pi / 2))
    extras.append(include('hatchback_red', 'coche_1', 14, -7, 0.0, math.pi / 2))
    for i, (x, y) in enumerate([(-50, 0), (50, 0), (-50, 40), (50, 45), (-50, 80), (50, 80)]):
        extras.append(lamp_post(f'poste_{i}', x, y, (x * 0.8, y), light=False))
    keep = [rect(X0 - 4, Y0 - 4, X1 + 4, Y1 + 4), rect(-6, -40, 6, -10)]
    extras.append(trees(51, 60, (-140, -120, 140, 170), keep, 9.0, 0.5))

    # --- La camioneta sospechosa ----------------------------------------------
    # VelocityControl con velocidad inicial: avanza a SUSPECT_SPEED y gira a
    # v/R, o sea da vueltas al anillo sin que nadie le mande nada (solo existe
    # en simulación). Arranca en el punto sur del anillo mirando al este.
    omega = SUSPECT_SPEED / RING_R
    suspect = f'''<model name="sospechoso">{pose(cx, cy - RING_R, 0.02)}
      <link name="cuerpo">
        <inertial><pose>0 0 0.8 0 0 0</pose><mass>1800</mass>
          <inertia><ixx>900</ixx><iyy>4500</iyy><izz>4800</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c">{pose(0, 0, 0.9)}<geometry>{box(5.4, 2.2, 1.6)}</geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
        <visual name="v">{pose(0, 0, 0, 0, 0, SUSPECT_YAW)}<geometry><mesh><uri>model://pickup/meshes/pickup.dae</uri></mesh></geometry></visual>
        <visual name="soporte">{pose(-1.2, 0, 1.6)}<geometry>{box(0.1, 0.1, 0.5)}</geometry>{color((0.15, 0.15, 0.15))}</visual>
        {aruco_visual(7, -1.2, 0, 1.92, 1.2)}
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <initial_linear>{SUSPECT_SPEED} 0 0</initial_linear><initial_angular>0 0 {omega:.5f}</initial_angular>
      </plugin>
    </model>'''

    body = '\n'.join([
        sun((-0.45, 0.4, -0.8), (1.0, 0.96, 0.88), 1.0),
        static_model('escena', [ground, yard]),
        suspect, *extras])
    (WORLDS / w).mkdir(parents=True, exist_ok=True)
    period = 2 * math.pi * RING_R / SUSPECT_SPEED
    (WORLDS / w / f'{w}.sdf').write_text(world_sdf(
        w, 'Zona de bodegas (misión 6: cazar a la camioneta sospechosa con la cámara).\n'
        '     Correr con el modelo x500_cam_abajo. La camioneta lleva el ArUco 7 en el\n'
        f'     techo y da vueltas al anillo vial a {SUSPECT_SPEED} m/s (una vuelta cada {period:.0f} s),\n'
        f'     centro NED norte {cy}, este {cx}, radio {RING_R} m. Los coches estacionados\n'
        '     llevan ArUco 3, 12 y 25: son señuelos. DICT_5X5_250, placas de 1.2 m.',
        SCENE_DAY, body, systems=CAMERA_WORLD_SYSTEMS))
    spawn_yaml(w, 'En la H azul junto a la caseta de la entrada.')


SUSPECT_YAW = -math.pi / 2    # giro de la malla de la pickup para que su frente quede en +x


def aruco_visual(marker_id, x, y, z, size):
    link = Link('tmp')
    aruco_plate(link, marker_id, x, y, z, size)
    return link.items[0].replace('name="v_1"', f'name="aruco_{marker_id}"')


# =============================================================================
# Mundo 6: base_carga (misión 7, patrulla y estación de carga)
# =============================================================================

STATION = (18.0, 30.0)   # la estación de carga (ENU)


def build_base_carga():
    w = 'base_carga'
    X0, X1, Y0, Y1 = -35.0, 35.0, -6.0, 46.0
    sx, sy = STATION

    pasto = Obj()
    pasto.plane(-150, -150, 150, 180, 0.0, 4.0)
    pasto.save(MALLAS / w / 'pasto.obj')
    patio = Obj()
    patio.plane(X0, Y0, X1, Y1, 0.01, 2.5)
    patio.save(MALLAS / w / 'patio.obj')
    acceso = Obj()
    acceso.road(-4, -40, 4, Y0, 0.02, along_x=False)
    acceso.save(MALLAS / w / 'acceso.obj')

    ground = Link('piso')
    ground.visual(mesh(w, 'pasto'), tex('pasto'))
    ground.visual(mesh(w, 'patio'), tex('grava', '0.85 0.85 0.85 1'))
    ground.visual(mesh(w, 'acceso'), tex('calle'))
    ground.collision(box(300, 330, 1), pose(0, 15, -0.5))
    helipad(ground, 0, 0, ring=(1.0, 0.85, 0.1))

    # --- La estación de carga -------------------------------------------------
    st = Link('estacion_carga')
    st.solid(box(3.4, 3.4, 0.25), color((0.18, 0.18, 0.2), rough=0.6), pose(sx, sy, 0.125))
    for i in range(16):                       # orilla amarillo/negro
        for side in range(4):
            t = -1.6 + 3.2 * (i + 0.5) / 16
            rgb = (0.95, 0.8, 0.05) if i % 2 == 0 else (0.05, 0.05, 0.05)
            px, py, bx, by = [(sx + t, sy - 1.6, 0.2, 0.2), (sx + t, sy + 1.6, 0.2, 0.2),
                              (sx - 1.6, sy + t, 0.2, 0.2), (sx + 1.6, sy + t, 0.2, 0.2)][side]
            st.visual(box(bx, by, 0.01), color(rgb), pose(px, py, 0.255), shadows=False)
    st.visual(box(2.6, 2.6, 0.01), color((0.95, 0.95, 0.95)), pose(sx, sy, 0.256), shadows=False)
    aruco_plate(st, 42, sx, sy, 0.262, 1.4)
    for dx in (-1.2, 1.2):                    # postes del cargador, con LEDs verdes
        st.solid(box(0.4, 0.3, 1.4), color((0.85, 0.87, 0.9), rough=0.3, metal=0.5), pose(sx + dx, sy + 2.3, 0.7))
        st.visual(box(0.25, 0.02, 0.6), color((0.1, 1.0, 0.3), glow=1.0), pose(sx + dx, sy + 2.14, 0.9), shadows=False)
    st.solid(box(1.2, 0.6, 1.0), color((0.25, 0.25, 0.28), rough=0.4, metal=0.6), pose(sx, sy + 3.2, 0.5))
    st.visual(box(0.6, 0.02, 0.3), color((0.1, 0.9, 0.3), glow=1.0), pose(sx, sy + 2.89, 0.75), shadows=False)

    # --- Edificios y equipo de la base ----------------------------------------
    base = Link('base')
    walls = Obj()
    walls.box(-20, 32, 2.25, 16, 10, 4.5, 2.0)            # comandancia
    base.collision(box(16, 10, 4.5), pose(-20, 32, 2.25))
    walls.save(MALLAS / w / 'comandancia.obj')
    base.visual(mesh(w, 'comandancia'), tex('ladrillo'))
    base.visual(box(16.6, 10.6, 0.3), color((0.3, 0.3, 0.32)), pose(-20, 32, 4.6))
    for x in (-25, -20, -15):
        base.visual(box(2.0, 0.06, 1.2), color((0.3, 0.45, 0.55), rough=0.05, metal=0.6), pose(x, 26.97, 2.6))
    hangar = Obj()
    warehouse(hangar, base, -20, 10, 14, 12, 6, door_side='E', roof_rgb=(0.4, 0.42, 0.45))
    hangar.save(MALLAS / w / 'hangar.obj')
    base.visual(mesh(w, 'hangar'), tex('concreto_claro', '0.85 0.85 0.8 1'))
    # Antena: mástil de 18 m con plato
    base.solid(cyl(0.25, 18), color((0.75, 0.75, 0.78), rough=0.3, metal=0.9), pose(-6, 22, 9))
    for z in (6, 12, 17):
        base.visual(box(1.6, 0.08, 0.08), color((0.75, 0.75, 0.78), metal=0.9), pose(-6, 22, z))
    base.visual(cyl(0.9, 0.15), color((0.9, 0.9, 0.92)), pose(-6.6, 22, 15, 0, 1.2, 0))
    base.visual(cyl(0.12, 0.3), color((1.0, 0.1, 0.1), glow=1.0), pose(-6, 22, 18.15), shadows=False)
    # Paneles solares: 3 filas inclinadas al sur
    for r in range(3):
        y = 6 + 3.2 * r
        for k in range(4):
            x = 2 + 3.3 * k
            base.solid(box(3.1, 1.8, 0.06), color((0.08, 0.12, 0.3), rough=0.15, metal=0.7),
                       pose(x, y, 1.3, -0.45, 0, 0))
            base.visual(box(0.1, 0.1, 1.1), color((0.6, 0.6, 0.62), metal=0.8), pose(x, y + 0.3, 0.55))
    fence(base, X0, Y0, X1, Y1, gate=(0.0, 8.0))
    base.visual(box(0.3, 0.3, 1.1), color((0.9, 0.85, 0.1)), pose(-4.4, Y0, 0.55))
    base.visual(box(7.0, 0.12, 0.12), color((0.95, 0.15, 0.1)), pose(-0.9, Y0, 1.0))

    extras = [include('suv', 'vehiculo_1', -10, 32, 0.0, math.pi / 2),
              include('pickup', 'vehiculo_2', -10, 38, 0.0, math.pi / 2),
              include('truck_delivery', 'vehiculo_3', -28, 1, 0.0, 0.0)]
    for i, (x, y) in enumerate([(X0 + 1, Y0 + 1), (X1 - 1, Y0 + 1), (X0 + 1, Y1 - 1), (X1 - 1, Y1 - 1)]):
        extras.append(lamp_post(f'poste_{i}', x, y, (x * 0.7, (y - 20) * 0.7 + 20), light=False))
    keep = [rect(X0 - 4, Y0 - 4, X1 + 4, Y1 + 4), rect(-6, -45, 6, -6)]
    extras.append(trees(61, 55, (-130, -120, 130, 160), keep, 9.0, 0.55))

    body = '\n'.join([
        sun((-0.45, 0.4, -0.8), (1.0, 0.96, 0.88), 1.0),
        static_model('escena', [ground, st, base]),
        *extras])
    (WORLDS / w).mkdir(parents=True, exist_ok=True)
    (WORLDS / w / f'{w}.sdf').write_text(world_sdf(
        w, 'Base con estación de carga (misión 7: patrullar y recargar).\n'
        '     Correr con el modelo x500_cam_abajo. El patio cercado va de NED norte\n'
        f'     {Y0}..{Y1}, este {X0}..{X1}. La estación de carga está en NED norte {sy}, este\n'
        f'     {sx}: plataforma de 3.4 m con el ArUco 42 (1.4 m, DICT_5X5_250) al centro.',
        SCENE_DAY, body, systems=CAMERA_WORLD_SYSTEMS))
    spawn_yaml(w, 'En la H de la entrada de la base.')


if __name__ == '__main__':
    build_plaza_show()
    build_plaza_show(concert=True)
    build_puente_rio()
    build_barrio_entrega()
    build_cueva()
    build_zona_bodegas()
    build_base_carga()
    print('Mundos escritos en', WORLDS)
