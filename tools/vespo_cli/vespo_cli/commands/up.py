"""
`vespo up` / `vespo down`: everything needed to fly, started and stopped together.

  --sim   agent over UDP + PX4 SITL in Gazebo + QGroundControl
  --real  agent over serial/USB to the flight controller + QGroundControl

Without either, the role decides: sim -> --sim, companion -> --real.
QGroundControl only starts on roles that use it (not on the companion).
How the processes are shown and stopped lives in session.py.
"""

import shlex
import sys
from typing import Annotated, Optional, Tuple

import typer

from .. import completion, procs, session, system, ui
from . import agent as agent_cmd
from . import load_config, require_px4, require_role
from . import qgc as qgc_cmd


def _vespo(cfg, *args, name, label=None):
    """A service running a vespo subcommand with this same installation."""
    return session.Service(name, [sys.executable, "-m", "vespo_cli", *args], cfg.ws,
                           label or shlex.join(["vespo", *args]))


def _usb_serial():
    """First USB serial port (a Pixhawk over USB is ttyACM), or None."""
    return next((p for p in system.serial_candidates() if "ttyACM" in p or "ttyUSB" in p), None)


def _mode(cfg, sim, real):
    if sim and real:
        ui.abort("Use either --sim or --real")
    if sim or real:
        return "sim" if sim else "real"
    role = require_role(cfg)
    spec = cfg.role_spec()
    if spec.get("simulation"):
        return "sim"
    if spec.get("link") == "serial":
        return "real"
    ui.abort(f"The {role} role flies nothing by itself. Pass --sim or --real")


def up(
    sim: Annotated[bool, typer.Option("--sim", help="Agent over UDP + PX4 SITL in Gazebo.")] = False,
    real: Annotated[bool, typer.Option("--real", help="Agent over serial/USB to the flight controller.")] = False,
    no_qgc: Annotated[bool, typer.Option("--no-qgc", help="Without QGroundControl.")] = False,
    serial: Annotated[Optional[str], typer.Option("--serial", "-s", metavar="DEV", help="Serial device for --real.", autocompletion=completion.serial_ports)] = None,
    headless: Annotated[Optional[bool], typer.Option("--headless/--gui", help="Without / with the Gazebo window (--sim).")] = None,
    world: Annotated[Optional[str], typer.Option("--world", "-w", help="PX4 or workspace world (--sim).", autocompletion=completion.worlds)] = None,
    custom: Annotated[Optional[str], typer.Option("--custom", metavar="MODEL", help="Any PX4 or workspace model (--sim).", autocompletion=completion.models)] = None,
    launch: Annotated[Optional[Tuple[str, str]], typer.Option("--launch", metavar="PKG FILE", help="Also run: ros2 launch PKG FILE.")] = None,
):
    """Start the agent, PX4 (SITL or real) and QGroundControl together."""
    cfg = load_config()
    mode = _mode(cfg, sim, real)

    state = session.load_state()
    alive = session.running(state)
    if alive:
        ui.abort(f"Already up ({', '.join(alive)}). Stop it first: vespo down")
    if state:
        session.stop(state)   # closes the panes a previous up left open
        session.clear_state()

    if mode == "real" and (headless is not None or world or custom):
        ui.abort("--headless, --gui, --world and --custom are only for --sim")
    if mode == "sim" and serial:
        ui.abort("--serial is only for --real")

    services = []
    if mode == "sim":
        require_px4(cfg)
        args = ["simulate"]
        if custom:
            args += ["--custom", custom]
        if world:
            args += ["--world", world]
        if headless is not None:
            args.append("--headless" if headless else "--gui")
        services.append(_vespo(cfg, *args, name="px4"))
        link_args, title = agent_cmd.link_args(cfg, "udp")
        services.append(_vespo(cfg, "agent", "--udp", "--port", link_args[2], name="agent"))
    else:
        device = serial or cfg.get("agent.device") or _usb_serial()
        if not device:
            ui.abort("No flight controller found on USB or serial. Connect it, or pass --serial DEV")
        link_args, title = agent_cmd.link_args(cfg, "serial", device)
        services.append(_vespo(cfg, "agent", "--serial", device, "--baud", link_args[4], name="agent"))

    if launch and all(launch):
        package, file = launch
        if not (cfg.ws / "install" / package).is_dir():
            ui.abort(f"Package {package} is not built. Run: vespo build {package}")
        ros_launch = shlex.join(["ros2", "launch", package, file])
        services.append(session.Service("launch", procs.ros_cmd(cfg.distro, ros_launch, ws=cfg.ws), cfg.ws, ros_launch))

    use_qgc = not no_qgc and bool(cfg.role_spec().get("qgc"))
    ui.header("Simulation" if mode == "sim" else f"Flight controller on {title}")

    # VESPO_UP tells the commands in the panes that up started their companions
    env = {"VESPO_WS": cfg.ws, "ROS_DOMAIN_ID": cfg.get("ros.domain_id", 0), "VESPO_UP": 1}
    in_kitty, why = session.kitty_available()
    if why:
        ui.warn(f"{why}. Running everything in this terminal")

    if in_kitty:
        state = session.start_kitty(services, env)
        if state is None:
            raise typer.Exit(1)
        state.update(mode=mode, ws=str(cfg.ws))
        _start_qgc(cfg, state, use_qgc)
        session.save_state(state)
        ui.info("Each process has its pane in the 'vespo' tab. Stop everything with: vespo down")
        return

    def started(state):
        state.update(mode=mode, ws=str(cfg.ws))
        _start_qgc(cfg, state, use_qgc)
        session.save_state(state)

    session.start_foreground(services, env, started)
    session.clear_state()
    if mode == "sim":
        session.kill_strays(["sim"])


def _start_qgc(cfg, state, use_qgc):
    if not use_qgc:
        return
    if qgc_cmd.running():
        ui.ok("QGroundControl is already running")
        return
    path = qgc_cmd.check(cfg)
    if path:
        state["procs"].append({"name": "qgc", "pid": qgc_cmd.start(path), "holder": False})


def down():
    """Stop everything vespo up started, leaving no PX4 or Gazebo behind."""
    state = session.load_state()
    ui.header("Stopping")
    if state:
        names = [p["name"] for p in state.get("procs", [])]
        alive = session.running(state)
        if alive:
            ui.info(f"Stopping {', '.join(alive)}")
        forced = session.stop(state)
        if forced:
            ui.warn(f"{', '.join(forced)} did not stop on Ctrl+C and had to be killed")
        stopped = [n for n in names if n in alive]
        if stopped:
            ui.ok(f"Stopped {', '.join(stopped)}")
        session.clear_state()
    else:
        ui.info("Nothing started by vespo up is running")

    # PX4 and Gazebo can outlive their parents (a closed pane, a crash)
    kinds = ["agent"] if state and state.get("mode") == "real" else ["sim", "agent"]
    strays = session.kill_strays(kinds)
    for cmdline in strays:
        ui.warn(f"Killed leftover: {cmdline[:100]}")
    ui.ok("Nothing left running")
