"""`vespo doctor`: checks everything this machine needs, with a fix for each problem."""

import os
import shutil
from pathlib import Path
from typing import Annotated

import typer
import yaml

from .. import bashrc, fx, msgs, procs, system, ui
from . import load_config

ROSDEP_LIST = Path("/etc/ros/rosdep/sources.list.d/20-default.list")


class Report:
    """Collects check results; failures make doctor exit 1."""

    def __init__(self, out):
        self.out = out
        self.failed = 0
        self.warned = 0

    def section(self, title):
        self.out.print("")
        ui.header(title)

    def ok(self, message):
        ui.ok(message)

    def warn(self, message, fix=None):
        self.warned += 1
        ui.warn(message)
        if fix:
            ui.console.print(f"    [dim]fix:[/] {fix}")

    def fail(self, message, fix=None):
        self.failed += 1
        ui.fail(message)
        if fix:
            ui.console.print(f"    [dim]fix:[/] {fix}")

    def check(self, passed, message, failure, fix=None, warn_only=False):
        if passed:
            self.ok(message)
        elif warn_only:
            self.warn(failure, fix)
        else:
            self.fail(failure, fix)
        return passed


def check_system(r, cfg):
    r.section("System")
    ui.info(system.describe_platform())
    distros = system.installed_ros_distros()
    r.check(cfg.distro in distros, f"ROS 2 {cfg.distro} installed",
            f"ROS 2 {cfg.distro} not installed (found: {', '.join(distros) or 'none'})",
            f"https://docs.ros.org/en/{cfg.distro}/Installation.html")
    for tool, pkg in (("git", "git"), ("colcon", "python3-colcon-common-extensions"),
                      ("vcs", "python3-vcstool"), ("rosdep", "python3-rosdep")):
        r.check(shutil.which(tool), f"{tool} found", f"{tool} not found", f"sudo apt install {pkg}")
    r.check(ROSDEP_LIST.is_file(), "rosdep initialized", "rosdep not initialized",
            "sudo rosdep init && rosdep update", warn_only=True)


def _pinned_repos(cfg):
    try:
        return (yaml.safe_load(cfg.repos_path.read_text()) or {}).get("repositories", {})
    except (OSError, yaml.YAMLError):
        return {}


def check_workspace(r, cfg):
    r.section("Workspace")
    ui.info(str(cfg.ws))
    r.check(cfg.role, f"Role: {cfg.role}", "No role set", "vespo init")

    env_ws = os.environ.get("VESPO_WS")
    if env_ws and Path(env_ws).resolve() != cfg.ws:
        r.warn(f"VESPO_WS points to {env_ws}, not this workspace", "open a new terminal, or: vespo init")
    env_domain = os.environ.get("ROS_DOMAIN_ID")
    domain = str(cfg.get("ros.domain_id", 0))
    if env_domain is not None and env_domain != domain:
        r.warn(f"ROS_DOMAIN_ID is {env_domain} in this shell, the workspace uses {domain}", "open a new terminal")
    r.check(bashrc.installed(), "Workspace set up in ~/.bashrc", "Workspace not in ~/.bashrc",
            "vespo init", warn_only=True)

    for name, spec in _pinned_repos(cfg).items():
        path = cfg.ws / "src" / name
        if not (path / ".git").exists():
            r.fail(f"{name} not cloned", "vespo init (or: vcs import src < vespo.repos)")
            continue
        wanted = system.git_resolve(path, str(spec.get("version", "")))
        head = system.git_head(path)
        r.check(wanted and head == wanted, f"{name} at the pinned version",
                f"{name} is not at the pinned version {spec.get('version')}",
                f"cd src/{name} && git fetch && git checkout {spec.get('version')}")

    skip = set(cfg.role_spec().get("skip", []))
    for pkg in ("px4_msgs", "microxrcedds_agent"):
        if pkg in skip:
            continue
        r.check((cfg.ws / "install" / pkg).is_dir(), f"{pkg} built", f"{pkg} not built", "vespo build")


def check_px4(r, cfg):
    r.section("PX4")
    path = cfg.get("px4.path")
    if not path or not system.is_px4_tree(path):
        r.fail("PX4-Autopilot path not set or invalid", "vespo config edit px4_path")
        return
    px4 = Path(path)
    head = system.git_head(px4)
    pinned = cfg.get("px4.commit")
    r.check(head == pinned, f"PX4-Autopilot at {cfg.get('px4.ref')} ({px4})",
            f"PX4-Autopilot is not at the pinned {cfg.get('px4.ref')}", "vespo px4 update " + str(pinned)[:10])

    repo = cfg.ws / "src" / "px4_msgs"
    if (repo / ".git").exists():
        missing, extra, changed = msgs.diff(msgs.message_blobs(px4, head) or {},
                                            msgs.message_blobs(repo, system.git_head(repo)) or {})
        r.check(not (missing or extra or changed), "px4_msgs matches PX4's messages",
                f"px4_msgs differs from PX4 ({len(missing)} missing, {len(extra)} extra, {len(changed)} changed)",
                "vespo px4 update")

    r.check((px4 / "build" / "px4_sitl_default" / "bin" / "px4").is_file(), "PX4 SITL built",
            "PX4 SITL not built yet", "vespo simulate (builds it on the first run)", warn_only=True)
    gz = system.gz_version()
    r.check(gz and gz.startswith("8."), f"Gazebo Harmonic {gz}",
            f"Gazebo Harmonic not found (gz sim {gz or 'missing'})", f"sudo apt install ros-{cfg.distro}-ros-gz")


def check_link(r, cfg):
    link = cfg.role_spec().get("link")
    if link not in ("udp", "serial"):
        return
    r.section("Link to PX4")
    if link == "udp":
        port = cfg.get("agent.udp_port", 8888)
        r.check(system.udp_port_free(port), f"UDP port {port} free",
                f"UDP port {port} busy (an agent may already be running)", warn_only=True)
        return
    device = cfg.get("agent.device")
    r.check(device and Path(device).exists(), f"Serial device {device} present",
            f"Serial device {device or '(not set)'} not found", "connect the flight controller, or: vespo config edit serial")
    r.check(system.in_group("dialout"), "Session in the dialout group", "Session not in the dialout group",
            "sudo usermod -aG dialout $USER, then log out and back in")


def check_qgc(r, cfg):
    from . import qgc   # the AppImage checks live with the qgc command

    r.section("QGroundControl")
    path = qgc.find(cfg)
    if not r.check(path, f"Installed at {path}", "Not installed", "vespo qgc install", warn_only=True):
        return
    for pkg in qgc.missing_packages():
        r.warn(f"{pkg} missing ({qgc.REQUIRED_PKGS[pkg]})", f"sudo apt install {pkg}")
    r.check(not qgc.modemmanager_installed(), "ModemManager not installed",
            "ModemManager installed, it can grab the flight controller's USB port", "sudo apt remove modemmanager",
            warn_only=True)


def check_live(r, cfg):
    r.section("Live")
    _, out = procs.capture(procs.ros_cmd(cfg.distro, "ros2 topic list", ws=cfg.ws), timeout=20)
    topics = [t for t in out.splitlines() if t.startswith("/fmu/")]
    r.check(topics, f"{len(topics)} /fmu topics visible", "No /fmu topics",
            "start the agent (vespo agent) and PX4 (vespo simulate or the drone)")


def doctor(live: Annotated[bool, typer.Option("--live", help="Also look for /fmu topics.")] = False):
    """Check ROS, the workspace, PX4, Gazebo and the link, with a fix for each problem."""
    cfg = load_config()
    with fx.status("Checking") as out:
        r = Report(out)
        check_system(r, cfg)
        check_workspace(r, cfg)
        if cfg.role_spec().get("simulation") or cfg.get("px4.path"):
            check_px4(r, cfg)
        check_link(r, cfg)
        if cfg.role_spec().get("qgc"):
            check_qgc(r, cfg)
        if live:
            out.update("Looking for /fmu topics")
            check_live(r, cfg)

    ui.console.print("")
    if r.failed:
        ui.fail(f"{r.failed} problem(s), {r.warned} warning(s)")
        raise typer.Exit(1)
    if r.warned:
        ui.warn(f"All good, {r.warned} warning(s)")
    else:
        ui.ok("All good")
