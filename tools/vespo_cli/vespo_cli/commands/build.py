"""`vespo build`: colcon build of what this machine's role needs."""

import shlex
from typing import Annotated, List, Optional

import typer

from .. import completion, procs, ui
from . import load_config

# colcon messages that mean a package was built before without --symlink-install
STALE_BUILD = ("failed to create symbolic link", "existing path cannot be removed")


def colcon_command(cfg, packages, role):
    cmd = ["colcon", "build", "--symlink-install"]
    if packages:
        return cmd + ["--packages-select", *packages]
    skip = cfg.role_spec(role).get("skip", []) if role else []
    if skip:
        cmd += ["--packages-skip", *skip]
    regex = cfg.get("build.skip_regex", [])
    if regex:
        cmd += ["--packages-skip-regex", *regex]
    return cmd


def run_colcon(cfg, cmd):
    """Runs a colcon command from the workspace root with ROS sourced. Returns its exit code."""
    ui.info(" ".join(cmd))
    return procs.run(procs.ros_cmd(cfg.distro, shlex.join(cmd)), cwd=cfg.ws)


def stale_packages(cfg):
    """Packages whose last build failed because of an old non-symlink build."""
    stale = []
    for log in (cfg.ws / "log" / "latest_build").glob("*/stderr.log"):
        text = log.read_text(errors="ignore")
        if any(marker in text for marker in STALE_BUILD):
            stale.append(log.parent.name)
    return sorted(stale)


def build(
    packages: Annotated[Optional[List[str]], typer.Argument(help="Only these packages.", autocompletion=completion.packages)] = None,
    role: Annotated[Optional[str], typer.Option("--role", help="Build for another role.", autocompletion=completion.roles)] = None,
):
    """colcon build --symlink-install, skipping what the role does not need."""
    cfg = load_config()
    role = role or cfg.role
    if role and role not in cfg.roles():
        ui.abort(f"Unknown role '{role}'. Roles: {', '.join(cfg.roles())}")

    ui.header(f"Building {' '.join(packages) if packages else 'the workspace'}" + (f" for {role}" if role else ""))
    code = run_colcon(cfg, colcon_command(cfg, packages, role))
    if code == 0:
        ui.ok("Build finished. New terminals pick it up; in this one run: source install/setup.bash")
        return

    stale = stale_packages(cfg)
    if stale:
        ui.warn(f"{', '.join(stale)} were built before without --symlink-install")
        ui.info(f"Fix: vespo clean {' '.join(stale)} -r")
    raise typer.Exit(code)
