"""
Every machine setting `vespo init` asks for, in one place: its key in
vespo.local.toml, the question that asks it and the parser that validates a
typed value. `vespo init` walks through all of them; `vespo config` shows,
sets or re-asks any single one without a full init.
"""

import difflib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import click
import typer
from rich.markup import escape

from . import system, ui

NONE = "none"  # flight controller choice meaning "not decided / not listed"


# ---------------------------------------------------------------------------
# Prompt helpers
# ---------------------------------------------------------------------------

def ask(question, default, explicit, yes, flag, type=None):
    """Value from the flag if given, the default under --yes, else a prompt."""
    if explicit is not None:
        return explicit
    if yes:
        if default is None:
            ui.abort(f"{flag} is required with --yes")
        return default
    return typer.prompt(question, default=default, type=type)


def confirm(question, explicit, yes, default):
    if explicit is not None:
        return explicit
    if yes:
        return default
    return typer.confirm(question, default=default)


def choose(title, options, default, explicit, yes, flag, required=True):
    """
    Numbered menu. options: [(id, label)]. Accepts the number or the id.
    With required=False, --yes and no default returns None instead of failing.
    """
    ids = [option_id for option_id, _ in options]
    if explicit is not None:
        if explicit not in ids:
            ui.abort(f"Unknown value '{explicit}' for {flag}. Options: {', '.join(ids)}")
        return explicit
    if yes:
        if default is None and required:
            ui.abort(f"{flag} is required with --yes")
        return default

    ui.console.print(f"\n[bold]{escape(title)}[/]")
    for n, (option_id, label) in enumerate(options, 1):
        mark = "[bold cyan]›[/]" if option_id == default else " "
        ui.console.print(f" {mark} [cyan]{n:>2}[/]  {escape(label)} [dim]{escape(option_id)}[/]")

    default_n = str(ids.index(default) + 1) if default in ids else None
    while True:
        answer = str(typer.prompt("Choose", default=default_n)).strip()
        if answer.isdigit() and 1 <= int(answer) <= len(ids):
            return ids[int(answer) - 1]
        if answer in ids:
            return answer
        ui.warn(f"Type a number from 1 to {len(ids)}")


# ---------------------------------------------------------------------------
# Questions: (cfg, explicit, yes) -> value, or None to leave the key unset
# ---------------------------------------------------------------------------

def ask_role(cfg, explicit, yes):
    suggested = cfg.role or ("companion" if system.board() in ("jetson", "raspberry") else "sim")
    options = [(name, spec.get("description", "")) for name, spec in cfg.roles().items()]
    return choose("Role of this machine", options, suggested, explicit, yes, "--role")


def ask_domain_id(cfg, explicit, yes):
    return ask("ROS_DOMAIN_ID", cfg.get("ros.domain_id", 0), explicit, yes, "--domain-id", click.IntRange(0, 232))


def ask_flight_controller(cfg, explicit, yes):
    boards = cfg.get("flight_controllers", {})
    options = [(fc_id, fc["name"]) for fc_id, fc in boards.items()]
    options.append((NONE, "None yet, or not in the list"))
    choice = choose(
        "Flight controller", options, cfg.get("flight_controller.model"),
        explicit, yes, "--flight-controller", required=False,
    )
    return None if choice == NONE else choice


def ask_px4_path(cfg, explicit, yes):
    candidates = system.px4_candidates(cfg.ws)
    default = cfg.get("px4.path") or (str(candidates[0]) if candidates else None)
    while True:
        answer = ask("Path to PX4-Autopilot", default, explicit and str(explicit), yes, "--px4-path")
        try:
            return parse_px4_path(cfg, answer)
        except ValueError as e:
            if explicit is not None or yes:
                ui.abort(str(e))
            ui.warn(str(e))
            default = None


def ask_serial(cfg, explicit, yes):
    found = system.serial_candidates()
    if explicit is None and not yes:
        ui.info(f"Serial ports found: {', '.join(found) or 'none'}")
    default = cfg.get("agent.device") or (found[0] if found else system.DEFAULT_SERIAL[system.board()])
    device = ask("Serial device to the flight controller", default, explicit, yes, "--serial")
    if not Path(device).exists():
        ui.warn(f"{device} does not exist right now, doctor will keep reporting it")
    return device


def ask_qgc_path(cfg, explicit, yes):
    from .commands import qgc   # late: qgc is a command module

    found = qgc.find(cfg)
    if explicit is None and not found:
        ui.info("QGroundControl not found, install it later with: vespo qgc install")
        return None
    while True:
        answer = ask("Path to the QGroundControl AppImage", str(found) if found else None,
                     explicit and str(explicit), yes, "--qgc-path")
        try:
            return parse_qgc_path(cfg, answer)
        except ValueError as e:
            if explicit is not None or yes:
                ui.abort(str(e))
            ui.warn(str(e))
            found = None


def ask_baud(cfg, explicit, yes):
    return ask("Baud rate", cfg.get("agent.baud", 921600), explicit, yes, "--baud", click.IntRange(min=1))


# Simulation defaults, used by `vespo simulate --default`

def _sim_catalog(cfg):
    """(models, worlds) this machine can simulate; empty while PX4 is not set up."""
    from .commands import simulate as sim   # late: simulate is a command module

    px4 = cfg.get("px4.path")
    if not px4 or not system.is_px4_tree(px4):
        return [], []
    px4 = Path(px4)
    return (sim.px4_models(px4) + list(sim.workspace_models(cfg.ws)),
            sim.px4_worlds(px4) + list(sim.workspace_worlds(cfg.ws)))


def _ask_from_catalog(title, key, fallback, names, cfg, explicit, yes, flag):
    default = cfg.get(key, fallback)
    if not names:   # PX4 not found yet: accept any name, simulate checks it later
        return ask(title, default, explicit, yes, flag)
    return choose(title, [(name, "") for name in names], default, explicit, yes, flag)


def ask_sim_model(cfg, explicit, yes):
    models, _ = _sim_catalog(cfg)
    return _ask_from_catalog("Default simulation model", "simulation.model", "x500",
                             models, cfg, explicit, yes, "--sim-model")


def ask_sim_world(cfg, explicit, yes):
    _, worlds = _sim_catalog(cfg)
    return _ask_from_catalog("Default simulation world", "simulation.world", "default",
                             worlds, cfg, explicit, yes, "--sim-world")


def _ask_parsed(question, key, fallback, parse, cfg, explicit, yes, flag):
    default = str(cfg.get(key, fallback))
    while True:
        answer = ask(question, default, explicit, yes, flag)
        try:
            return parse(cfg, answer)
        except ValueError as e:
            if explicit is not None or yes:
                ui.abort(str(e))
            ui.warn(str(e))


def ask_sim_pose(cfg, explicit, yes):
    return _ask_parsed("Default spawn pose: auto (world's spawn.yaml) or x,y[,z[,yaw_deg]]",
                       "simulation.pose", "auto", parse_sim_pose, cfg, explicit, yes, "--sim-pose")


def ask_sim_follow(cfg, explicit, yes):
    return _ask_parsed("Default camera: auto (PX4's), off (free) or distance in metres",
                       "simulation.follow", "auto", parse_sim_follow, cfg, explicit, yes, "--sim-follow")


def ask_sim_headless(cfg, explicit, yes):
    return confirm("Run simulations without the Gazebo window by default?",
                   explicit, yes, bool(cfg.get("simulation.headless", False)))


# ---------------------------------------------------------------------------
# Parsers for `vespo config set NAME VALUE`: (cfg, text) -> value, ValueError if invalid
# ---------------------------------------------------------------------------

def parse_role(cfg, text):
    if text not in cfg.roles():
        raise ValueError(f"Unknown role '{text}'. Roles: {', '.join(cfg.roles())}")
    return text


def parse_int(low, high):
    def parse(cfg, text):
        try:
            value = int(text)
        except ValueError:
            raise ValueError(f"'{text}' is not a number") from None
        if not low <= value <= high:
            raise ValueError(f"Must be between {low} and {high}")
        return value
    return parse


def parse_flight_controller(cfg, text):
    boards = cfg.get("flight_controllers", {})
    if text not in boards:
        raise ValueError(f"Unknown flight controller '{text}'. Options: {', '.join(boards)}")
    return text


def parse_px4_path(cfg, text):
    path = Path(text).expanduser().resolve()
    if not system.is_px4_tree(path):
        raise ValueError(f"{path} has no msg/ or Tools/simulation/gz/, is it PX4-Autopilot?")
    return str(path)


def parse_qgc_path(cfg, text):
    path = Path(text).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"{path} does not exist. Install QGroundControl with: vespo qgc install")
    return str(path)


def parse_text(cfg, text):
    return text


def _parse_catalog_name(kind, index):
    def parse(cfg, text):
        names = _sim_catalog(cfg)[index]
        if names and text not in names:
            close = difflib.get_close_matches(text, names, n=3)
            hint = f" Did you mean: {', '.join(close)}?" if close else ""
            raise ValueError(f"Unknown {kind} '{text}'.{hint} List them with: vespo simulate --show")
        return text
    return parse


parse_sim_model = _parse_catalog_name("model", 0)
parse_sim_world = _parse_catalog_name("world", 1)


def parse_sim_pose(cfg, text):
    from .commands.simulate import pose_values

    text = str(text).strip()
    if text.lower() == "auto":
        return "auto"
    return ",".join(f"{v:g}" for v in pose_values(text))


def parse_sim_follow(cfg, text):
    from .commands.simulate import follow_value

    return follow_value(text)


def parse_bool(cfg, text):
    value = str(text).strip().lower()
    if value in ("true", "yes", "y", "1", "on"):
        return True
    if value in ("false", "no", "n", "0", "off"):
        return False
    raise ValueError(f"Expected true or false, got '{text}'")


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

@dataclass
class Setting:
    name: str        # what the user types: vespo config set <name> ...
    key: str         # dotted key in vespo.local.toml
    label: str
    ask: Callable
    parse: Callable
    applies: Callable = lambda cfg, role: True  # does this role use it?

    def describe(self, cfg):
        value = cfg.get(self.key)
        if value is None:
            return "not set"
        if self.name == "flight_controller":
            fc = cfg.get("flight_controllers", {}).get(value)
            return f"{fc['name']} ({fc['target']})" if fc else f"{value} (unknown)"
        return str(value)


def _simulation(cfg, role):
    return bool(cfg.role_spec(role).get("simulation"))


def _serial(cfg, role):
    return cfg.role_spec(role).get("link") == "serial"


def _qgc(cfg, role):
    return bool(cfg.role_spec(role).get("qgc"))


# Asked in this order by init; role first because the rest depend on it
SETTINGS = {s.name: s for s in [
    Setting("role", "machine.role", "Role", ask_role, parse_role),
    Setting("domain_id", "ros.domain_id", "ROS_DOMAIN_ID", ask_domain_id, parse_int(0, 232)),
    Setting("flight_controller", "flight_controller.model", "Flight controller",
            ask_flight_controller, parse_flight_controller),
    Setting("px4_path", "px4.path", "PX4-Autopilot path", ask_px4_path, parse_px4_path, _simulation),
    Setting("serial", "agent.device", "Serial device", ask_serial, parse_text, _serial),
    Setting("baud", "agent.baud", "Serial baud rate", ask_baud, parse_int(1, 4_000_000), _serial),
    Setting("qgc_path", "qgc.path", "QGroundControl AppImage", ask_qgc_path, parse_qgc_path, _qgc),
    # After px4_path: the model and world menus are read from the PX4 checkout
    Setting("sim_model", "simulation.model", "Default sim model", ask_sim_model, parse_sim_model, _simulation),
    Setting("sim_world", "simulation.world", "Default sim world", ask_sim_world, parse_sim_world, _simulation),
    Setting("sim_pose", "simulation.pose", "Default sim spawn pose", ask_sim_pose, parse_sim_pose, _simulation),
    Setting("sim_follow", "simulation.follow", "Default sim camera", ask_sim_follow, parse_sim_follow, _simulation),
    Setting("sim_headless", "simulation.headless", "Sim without Gazebo window", ask_sim_headless, parse_bool,
            _simulation),
]}


# Settings a role can work without; init and config leave them unset if skipped
OPTIONAL = ("flight_controller", "qgc_path")


def missing_for_role(cfg, role):
    """Settings the role uses that have no value yet."""
    return [s for s in SETTINGS.values()
            if s.applies(cfg, role) and cfg.get(s.key) is None and s.name not in OPTIONAL]
