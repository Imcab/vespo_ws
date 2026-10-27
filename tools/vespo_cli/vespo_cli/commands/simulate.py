"""
`vespo simulate`: PX4 SITL in Gazebo Harmonic.

PX4 worlds and models are launched the way PX4 documents it,
`make px4_sitl gz_<model>` with PX4_GZ_* variables. A world installed by a
workspace package (share/<pkg>/worlds/<world>.sdf) is started by vespo first;
PX4 then finds the running world and spawns the vehicle into it.
"""

import math
import os
import re
import subprocess
import time
from pathlib import Path
from typing import Annotated, Optional

import typer
import yaml
from rich.table import Table

from .. import completion, procs, ui
from . import load_config, require_px4

AIRFRAME = re.compile(r"^\d+_gz_(.+)$")
FOLLOW_DEFAULT = 2.0  # PX4's own camera offset, in metres


# ---------------------------------------------------------------------------
# Catalog
# ---------------------------------------------------------------------------

def _airframe_dirs(px4):
    # The build copy includes airframes added since the last ROMFS change
    return [px4 / "build" / "px4_sitl_default" / "etc" / "init.d-posix" / "airframes",
            px4 / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes"]


def px4_airframes(px4):
    """Models PX4 has an SITL airframe for: what `make px4_sitl gz_<model>` accepts."""
    names = set()
    for folder in _airframe_dirs(Path(px4)):
        for path in folder.glob("*_gz_*"):
            match = AIRFRAME.match(path.name)
            if match:
                names.add(match.group(1))
    return names


def px4_models(px4):
    """PX4 Gazebo models that also have an airframe, so they can fly."""
    models = Path(px4) / "Tools" / "simulation" / "gz" / "models"
    flyable = px4_airframes(px4)
    return sorted(p.name for p in models.iterdir() if p.name in flyable and (p / "model.sdf").is_file())


def px4_worlds(px4):
    return sorted(p.stem for p in (Path(px4) / "Tools" / "simulation" / "gz" / "worlds").glob("*.sdf"))


def _share_dirs(ws):
    return sorted((ws / "install").glob("*/share/*"))


def workspace_models(ws):
    """{model: models folder} installed by workspace packages."""
    found = {}
    for share in _share_dirs(ws):
        for sdf in (share / "models").glob("*/model.sdf"):
            found.setdefault(sdf.parent.name, sdf.parent.parent)
    return found


def workspace_worlds(ws):
    """{world: sdf path} installed by workspace packages.

    A world is either worlds/<world>.sdf or worlds/<world>/<world>.sdf, the
    layout tools/aruco uses so each world keeps its spawn.yaml and tags.yaml.
    """
    found = {}
    for share in _share_dirs(ws):
        for sdf in [*(share / "worlds").glob("*.sdf"), *(share / "worlds").glob("*/*.sdf")]:
            if sdf.parent.name in ("worlds", sdf.stem):
                found.setdefault(sdf.stem, sdf)
    return found


def workspace_airframes(ws):
    """{model: airframe file} shipped next to workspace models (<model>/<id>_gz_<model>)."""
    found = {}
    for model, models_dir in workspace_models(ws).items():
        for path in (models_dir / model).glob(f"*_gz_{model}"):
            if AIRFRAME.match(path.name):
                found.setdefault(model, path)
    return found


# ---------------------------------------------------------------------------
# Value parsers, shared with `vespo config`
# ---------------------------------------------------------------------------

def pose_values(text):
    """'x,y[,z[,yaw_deg]]' -> [x, y, z, yaw_deg]. Raises ValueError."""
    parts = [p.strip() for p in str(text).split(",") if p.strip()]
    if not 2 <= len(parts) <= 4:
        raise ValueError(f"Pose '{text}' must be x,y[,z[,yaw_deg]]")
    try:
        values = [float(p) for p in parts]
    except ValueError:
        raise ValueError(f"Pose '{text}' has a value that is not a number") from None
    return values + [0.0] * (4 - len(values))


def follow_value(text):
    """'auto', 'off' or a distance in metres (> 0). Raises ValueError."""
    value = str(text).strip().lower()
    if value in ("auto", "off"):
        return value
    try:
        metres = float(value)
    except ValueError:
        raise ValueError(f"Camera follow must be auto, off or metres, got '{text}'") from None
    if metres <= 0:
        raise ValueError("Camera follow distance must be greater than 0")
    return metres


def _spawn_pose(world_sdf):
    """Pose from <world>.spawn.yaml or spawn.yaml next to a workspace world."""
    for name in (f"{world_sdf.stem}.spawn.yaml", "spawn.yaml"):
        path = world_sdf.parent / name
        if path.is_file():
            data = yaml.safe_load(path.read_text()) or {}
            return [float(data.get(k, 0.0)) for k in ("x", "y", "z", "yaw_deg")]
    return None


def _world_name(sdf):
    """The name Gazebo gives the world: <world name="..."> in the file."""
    match = re.search(r'<world\s+name\s*=\s*"([^"]+)"', sdf.read_text(errors="ignore"))
    return match.group(1) if match else sdf.stem


# ---------------------------------------------------------------------------
# Command
# ---------------------------------------------------------------------------

def show(cfg, px4):
    table = Table(title="Simulation catalog", title_justify="left")
    table.add_column("Models", style="cyan")
    table.add_column("Worlds", style="green")
    models = px4_models(px4) + [f"{m} (workspace)" for m in workspace_models(cfg.ws)]
    worlds = px4_worlds(px4) + [f"{w} (workspace)" for w in workspace_worlds(cfg.ws)]
    for i in range(max(len(models), len(worlds))):
        table.add_row(models[i] if i < len(models) else "", worlds[i] if i < len(worlds) else "")
    ui.console.print(table)
    ui.info(f"Default: {cfg.get('simulation.model')} in {cfg.get('simulation.world')} "
            "(change with: vespo config edit sim_model / sim_world)")


def simulate(
    minimum: Annotated[bool, typer.Option("--minimum", help="x500 in PX4's default world.")] = False,
    default: Annotated[bool, typer.Option("--default", help="This machine's sim_* settings (the default).")] = False,
    custom: Annotated[Optional[str], typer.Option("--custom", metavar="MODEL", help="Any PX4 or workspace model.", autocompletion=completion.models)] = None,
    show_catalog: Annotated[bool, typer.Option("--show", help="List models and worlds, then exit.")] = False,
    world: Annotated[Optional[str], typer.Option("--world", "-w", help="PX4 or workspace world.", autocompletion=completion.worlds)] = None,
    headless: Annotated[Optional[bool], typer.Option("--headless/--gui", help="Without / with the Gazebo window.")] = None,
    pose: Annotated[Optional[str], typer.Option("--pose", metavar="X,Y[,Z[,YAW]]", help="Spawn point, yaw in degrees.")] = None,
    follow: Annotated[Optional[float], typer.Option("--follow", metavar="METRES", help="Camera follows the vehicle.")] = None,
    no_follow: Annotated[bool, typer.Option("--no-follow", help="Free camera.")] = False,
):
    """Run PX4 SITL in Gazebo Harmonic."""
    cfg = load_config()
    px4 = require_px4(cfg)

    if show_catalog:
        show(cfg, px4)
        return
    if sum([minimum, default, custom is not None]) > 1:
        ui.abort("Use only one of --minimum, --default, --custom")

    if minimum:
        model, world_name, pose_text, follow_setting, run_headless = "x500", "default", "auto", "auto", False
    else:
        model = custom or cfg.get("simulation.model", "x500")
        world_name = cfg.get("simulation.world", "default")
        pose_text = str(cfg.get("simulation.pose", "auto"))
        follow_setting = cfg.get("simulation.follow", "auto")
        run_headless = bool(cfg.get("simulation.headless", False))
    world_name = world or world_name
    run_headless = run_headless if headless is None else headless
    if no_follow:
        follow_setting = "off"
    elif follow is not None:
        follow_setting = follow_value(follow)

    ws_models, ws_worlds = workspace_models(cfg.ws), workspace_worlds(cfg.ws)
    px4_world_list = px4_worlds(px4)
    if world_name not in px4_world_list and world_name not in ws_worlds:
        ui.abort(f"Unknown world '{world_name}'. List them with: vespo simulate --show")

    ws_airframe = workspace_airframes(cfg.ws).get(model)
    if ws_airframe is None and model not in px4_airframes(px4):
        ui.abort(f"PX4 has no SITL airframe for '{model}' (a *_gz_{model} file in "
                 "ROMFS/px4fmu_common/init.d-posix/airframes or next to the workspace model). "
                 "List flyable models with: vespo simulate --show")
    models_dir = ws_models.get(model, px4 / "Tools" / "simulation" / "gz" / "models")
    if not (models_dir / model / "model.sdf").is_file():
        ui.abort(f"No Gazebo model '{model}' in PX4 or in the workspace. Did you run vespo build?")

    # Pose: explicit flag, else the setting, else the world's spawn file
    world_sdf = ws_worlds.get(world_name)
    try:
        if pose is not None:
            spawn = pose_values(pose)
        elif pose_text.lower() != "auto":
            spawn = pose_values(pose_text)
        else:
            spawn = (_spawn_pose(world_sdf) if world_sdf else None)
    except ValueError as e:
        ui.abort(str(e))

    env = {"PX4_GZ_MODELS": models_dir}
    if spawn:
        x, y, z, yaw = spawn
        env["PX4_GZ_MODEL_POSE"] = f"{x:g},{y:g},{z:g},0,0,{math.radians(yaw):.4f}"
    if run_headless:
        env["HEADLESS"] = 1
    if follow_setting != "off" and not run_headless:
        metres = FOLLOW_DEFAULT if follow_setting == "auto" else float(follow_setting)
        env.update(PX4_GZ_FOLLOW=1, PX4_GZ_FOLLOW_OFFSET_X=-metres, PX4_GZ_FOLLOW_OFFSET_Y=-metres,
                   PX4_GZ_FOLLOW_OFFSET_Z=metres)

    ui.header(f"Simulating {model} in {world_name}")
    if spawn:
        ui.info(f"Spawn pose x={spawn[0]:g} y={spawn[1]:g} z={spawn[2]:g} yaw={spawn[3]:g}°")
    ui.info("Gazebo window off" if run_headless else "Gazebo window on")
    if not os.environ.get("VESPO_UP"):
        ui.info("Start the agent in another terminal: vespo agent")

    # A workspace world loads PX4's Gazebo plugins, and a workspace airframe
    # is linked into the build, so both need the build done before starting
    if world_sdf or ws_airframe:
        code = _build_sitl(px4)
        if code:
            raise typer.Exit(code)

    gz_procs = []
    if world_sdf:
        gz_procs = _start_world(cfg, px4, world_sdf, run_headless, models_dir)
        env["PX4_GZ_WORLD"] = _world_name(world_sdf)
    else:
        env["PX4_GZ_WORLD"] = world_name

    try:
        if ws_airframe:
            code = _run_workspace_airframe(px4, model, ws_airframe, env)
        else:
            code = procs.run(["make", "px4_sitl", f"gz_{model}"], cwd=px4, env=env)
    finally:
        for proc in gz_procs:
            if proc.poll() is None:
                proc.terminate()
        for proc in gz_procs:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
    raise typer.Exit(code)


def _sitl_build(px4):
    return px4 / "build" / "px4_sitl_default"


def _build_sitl(px4):
    """Builds PX4 SITL and its Gazebo plugins, what `make px4_sitl gz_<model>` builds."""
    build = _sitl_build(px4)
    if not (build / "CMakeCache.txt").is_file():
        code = procs.run(["make", "px4_sitl"], cwd=px4)
        if code:
            return code
    return procs.run(["cmake", "--build", str(build), "--target", "px4", "px4_gz_plugins"], cwd=px4)


def _run_workspace_airframe(px4, model, airframe, env):
    """
    Runs PX4 SITL on an airframe that lives in the workspace, not in PX4.

    `make px4_sitl gz_<model>` only exists for airframes in PX4's ROMFS, and
    every ROMFS change regenerates build/.../etc, so the airframe is linked in
    after building and PX4 is started the way that make target starts it.
    """
    build = _sitl_build(px4)
    link = build / "etc" / "init.d-posix" / "airframes" / airframe.name
    link.unlink(missing_ok=True)
    link.symlink_to(airframe.resolve())
    return procs.run([str(build / "bin" / "px4")], cwd=build / "rootfs",
                     env={**env, "PX4_SIM_MODEL": f"gz_{model}", "GZ_IP": "127.0.0.1"})


def _start_world(cfg, px4, world_sdf, run_headless, models_dir):
    """Starts Gazebo on a workspace world and waits until it publishes its clock."""
    gz_root = px4 / "Tools" / "simulation" / "gz"
    paths = [str(models_dir), str(gz_root / "models"), str(gz_root / "worlds"), str(world_sdf.parent)]
    paths += [str(p) for p in workspace_models(cfg.ws).values()]
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])
    # What PX4's gz_env.sh sets: without its server.config a world with no
    # <plugin> of its own gets no IMU, barometer, magnetometer or GPS systems
    plugins = [str(_sitl_build(px4) / "src" / "modules" / "simulation" / "gz_plugins")]
    if os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH"):
        plugins.append(os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"])
    env = {**os.environ, "GZ_SIM_RESOURCE_PATH": ":".join(dict.fromkeys(paths)),
           "GZ_SIM_SYSTEM_PLUGIN_PATH": ":".join(plugins),
           "GZ_SIM_SERVER_CONFIG_PATH": str(px4 / "src" / "modules" / "simulation" / "gz_bridge" / "server.config")}

    ui.info(f"Starting workspace world {world_sdf}")
    started = [subprocess.Popen(["gz", "sim", "-r", "-s", str(world_sdf)], env=env,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)]
    name = _world_name(world_sdf)
    for _ in range(60):
        _, out = procs.capture(["gz", "topic", "-l"], timeout=5)
        if f"/world/{name}/clock" in out:
            break
        if started[0].poll() is not None:
            ui.abort(f"Gazebo could not load {world_sdf}")
        time.sleep(0.5)
    else:
        started[0].terminate()
        ui.abort(f"Gazebo did not start world '{name}' within 30 s")

    if not run_headless:
        started.append(subprocess.Popen(["gz", "sim", "-g"], env=env,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    return started
