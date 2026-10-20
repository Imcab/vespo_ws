"""Detection of the host: board, OS, installed tools, PX4 checkouts and serial ports."""

import grp
import os
import platform
import re
import shutil
import socket
from pathlib import Path

from .procs import capture


def os_release():
    """Parses /etc/os-release into a dict (PRETTY_NAME, VERSION_ID, ...)."""
    data = {}
    try:
        lines = Path("/etc/os-release").read_text().splitlines()
    except OSError:
        return data
    for line in lines:
        key, sep, value = line.partition("=")
        if sep:
            data[key] = value.strip().strip('"')
    return data


def board():
    """Returns 'jetson', 'raspberry' or 'pc'."""
    try:
        model = Path("/proc/device-tree/model").read_text(errors="ignore")
    except OSError:
        model = ""
    if Path("/etc/nv_tegra_release").exists() or "Jetson" in model:
        return "jetson"
    if "Raspberry Pi" in model:
        return "raspberry"
    return "pc"


def describe_platform():
    name = os_release().get("PRETTY_NAME", "unknown OS")
    return f"{name} · {platform.machine()} · {board()}"


def installed_ros_distros():
    return sorted(p.name for p in Path("/opt/ros").glob("*") if (p / "setup.bash").is_file())


def gz_version():
    """Returns the gz-sim version string, e.g. '8.15.0', or None if absent."""
    if not shutil.which("gz"):
        return None
    _, out = capture(["gz", "sim", "--version"], timeout=20)
    match = re.search(r"version (\d+\.\d+\.\d+)", out)
    return match.group(1) if match else None


# ---------------------------------------------------------------------------
# PX4
# ---------------------------------------------------------------------------

def is_px4_tree(path):
    path = Path(path)
    return (path / "msg").is_dir() and (path / "Tools" / "simulation" / "gz").is_dir()


def px4_candidates(ws):
    """Existing PX4-Autopilot checkouts in the usual places, best guess first."""
    places = [
        ws.parent / "PX4-Autopilot",
        Path.home() / "PX4-Autopilot",
        Path.home() / "src" / "PX4-Autopilot",
        Path("/opt/PX4-Autopilot"),
    ]
    found = []
    for place in places:
        if is_px4_tree(place) and place.resolve() not in found:
            found.append(place.resolve())
    return found


def px4_board_file(px4, target):
    """
    The .px4board file a make target builds, e.g. px4_fmu-v6c_default ->
    boards/px4/fmu-v6c/default.px4board. Vendor and variant never contain
    underscores; the model may (it is everything in between).
    """
    vendor, _, rest = target.partition("_")
    model, _, variant = rest.rpartition("_")
    return Path(px4) / "boards" / vendor / model / f"{variant}.px4board"


# ---------------------------------------------------------------------------
# Git
# ---------------------------------------------------------------------------

def git_head(repo):
    code, out = capture(["git", "-C", str(repo), "rev-parse", "HEAD"])
    return out if code == 0 else None


def git_resolve(repo, ref):
    """Commit hash a branch, tag or (partial) hash points to, or None."""
    code, out = capture(["git", "-C", str(repo), "rev-parse", "--verify", "--quiet", f"{ref}^{{commit}}"])
    return out if code == 0 else None


# ---------------------------------------------------------------------------
# Serial and network
# ---------------------------------------------------------------------------

# Jetson UARTs, Raspberry Pi UARTs, then USB (Pixhawk over USB is ttyACM)
SERIAL_PATTERNS = ["ttyTHS*", "serial[0-9]", "ttyAMA*", "ttyACM*", "ttyUSB*"]

DEFAULT_SERIAL = {
    "jetson": "/dev/ttyTHS1",
    "raspberry": "/dev/serial0",
    "pc": "/dev/ttyACM0",
}


def serial_candidates():
    found = []
    for pattern in SERIAL_PATTERNS:
        found += sorted(str(p) for p in Path("/dev").glob(pattern))
    return found


def udp_port_free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind(("0.0.0.0", port))
        except OSError:
            return False
    return True


def in_group(name):
    """True if the current session (not just the user database) has the group."""
    try:
        gid = grp.getgrnam(name).gr_gid
    except KeyError:
        return False
    return gid == os.getgid() or gid in os.getgroups()
