"""
Values offered on Tab: packages, roles, settings, models, worlds, boards,
serial ports and PX4 versions. Bash calls these on every Tab press, so each
one is fast, reads only local files, and returns nothing instead of raising.
"""

import re
from functools import wraps

import typer

from . import config, system

PACKAGE_NAME = re.compile(r"<name>\s*([^<\s]+)\s*</name>")


def _safe(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return list(func(*args, **kwargs))
        except Exception:   # a broken config must never break the shell
            return []
    return wrapper


def _matching(values, incomplete):
    seen = []
    for value in values:
        name = value[0] if isinstance(value, tuple) else value
        if name.startswith(incomplete) and name not in seen:
            seen.append(name)
            yield value


def _cfg():
    return config.load()


@_safe
def packages(incomplete: str):
    """ROS packages in src/ and already built in install/."""
    cfg = _cfg()
    names = []
    for xml in (cfg.ws / "src").glob("**/package.xml"):
        if "COLCON_IGNORE" not in {p.name for p in xml.parent.iterdir()}:
            match = PACKAGE_NAME.search(xml.read_text(errors="ignore"))
            if match:
                names.append(match.group(1))
    names += [p.name for p in (cfg.ws / "install").iterdir() if p.is_dir()] if (cfg.ws / "install").is_dir() else []
    return _matching(sorted(names), incomplete)


@_safe
def roles(incomplete: str):
    return _matching([(name, spec.get("description", "")) for name, spec in _cfg().roles().items()], incomplete)


@_safe
def setting_names(incomplete: str):
    from .settings import SETTINGS
    return _matching([(s.name, s.label) for s in SETTINGS.values()], incomplete)


@_safe
def flight_controllers(incomplete: str):
    boards = _cfg().get("flight_controllers", {})
    return _matching([(fc_id, fc["name"]) for fc_id, fc in boards.items()], incomplete)


@_safe
def targets(incomplete: str):
    boards = _cfg().get("flight_controllers", {})
    return _matching(sorted({fc["target"] for fc in boards.values()}), incomplete)


@_safe
def serial_ports(incomplete: str):
    return _matching(system.serial_candidates(), incomplete)


def _catalog():
    from .commands import simulate as sim
    cfg = _cfg()
    px4 = cfg.get("px4.path")
    if not px4 or not system.is_px4_tree(px4):
        return [], []
    return (sim.px4_models(px4) + list(sim.workspace_models(cfg.ws)),
            sim.px4_worlds(px4) + list(sim.workspace_worlds(cfg.ws)))


@_safe
def models(incomplete: str):
    return _matching(_catalog()[0], incomplete)


@_safe
def worlds(incomplete: str):
    return _matching(_catalog()[1], incomplete)


@_safe
def px4_refs(incomplete: str):
    """Stable PX4 tags, newest first, plus the development branch."""
    from .commands.px4 import STABLE_TAG
    from .procs import capture
    px4 = _cfg().get("px4.path")
    _, out = capture(["git", "-C", str(px4), "tag", "-l", "v*"], timeout=3)
    tags = sorted((t for t in out.split() if STABLE_TAG.match(t)),
                  key=lambda t: tuple(int(n) for n in STABLE_TAG.match(t).groups()), reverse=True)
    return _matching([*tags, "origin/main"], incomplete)


@_safe
def setting_values(ctx: typer.Context, incomplete: str):
    """Values for `vespo config set NAME <Tab>`, depending on NAME."""
    name = ctx.params.get("name")
    fixed = {
        "role": roles, "flight_controller": flight_controllers, "serial": serial_ports,
        "sim_model": models, "sim_world": worlds,
    }
    if name in fixed:
        return fixed[name](incomplete)
    choices = {
        "sim_headless": ["true", "false"],
        "sim_follow": ["auto", "off", "2", "4"],
        "sim_pose": ["auto"],
        "baud": ["921600", "460800", "115200", "57600"],
        "domain_id": ["0"],
    }
    if name in choices:
        return _matching(choices[name], incomplete)
    if name in ("px4_path", "qgc_path"):
        return []   # empty list: bash falls back to completing file paths
    return []
