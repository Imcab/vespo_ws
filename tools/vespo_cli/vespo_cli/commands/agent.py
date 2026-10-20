"""`vespo agent`: the Micro XRCE-DDS Agent that bridges PX4 and ROS 2."""

import shlex
from pathlib import Path
from typing import Annotated, Optional

import typer

from .. import completion, procs, system, ui
from . import load_config


def link_args(cfg, link, serial=None, baud=None, port=None):
    """
    MicroXRCEAgent arguments for a link ("udp" or "serial") and a title for
    it, after checking the port is free or the device exists. Aborts with the fix.
    """
    if link == "udp":
        port = port or cfg.get("agent.udp_port", 8888)
        if not system.udp_port_free(port):
            ui.abort(f"UDP port {port} is busy: another agent is probably running")
        args, title = ["udp4", "-p", str(port)], f"UDP {port}"
    else:
        device = serial or cfg.get("agent.device")
        if not device:
            ui.abort("No serial device set. Run: vespo config edit serial")
        if not Path(device).exists():
            ui.abort(f"{device} does not exist. Is the flight controller connected? Ports found: "
                     f"{', '.join(system.serial_candidates()) or 'none'}")
        if not system.in_group("dialout"):
            ui.warn("This session is not in the dialout group; opening the port may fail (log out and back in)")
        baud = baud or cfg.get("agent.baud", 921600)
        args, title = ["serial", "--dev", device, "-b", str(baud)], f"{device} at {baud} baud"

    if not (cfg.ws / "install" / "microxrcedds_agent").is_dir():
        ui.abort("The agent is not built in this workspace. Run: vespo build")
    return args, title


def agent(
    serial: Annotated[Optional[str], typer.Option("--serial", "-s", metavar="DEV", help="Serial device.", autocompletion=completion.serial_ports)] = None,
    udp: Annotated[bool, typer.Option("--udp", help="UDP, as for the simulator.")] = False,
    baud: Annotated[Optional[int], typer.Option("--baud", "-b", help="Serial baud rate.")] = None,
    port: Annotated[Optional[int], typer.Option("--port", "-p", help="UDP port.")] = None,
):
    """Run the Micro XRCE-DDS Agent (UDP for sim, serial for companion)."""
    cfg = load_config()
    if serial and udp:
        ui.abort("Use either --serial or --udp")

    link = "serial" if serial else "udp" if udp else cfg.role_spec().get("link")
    if not link:
        ui.abort("This machine has no role yet. Run vespo init, or pass --udp / --serial DEV")
    if link == "none":
        ui.abort(f"The {cfg.role} role does not talk to PX4. Pass --udp or --serial DEV to run it anyway")

    args, title = link_args(cfg, link, serial, baud, port)
    ui.header(f"Micro XRCE-DDS Agent on {title}")
    ui.info("Ctrl+C to stop")
    code = procs.run(procs.ros_cmd(cfg.distro, shlex.join(["MicroXRCEAgent", *args]), ws=cfg.ws))
    raise typer.Exit(code)
