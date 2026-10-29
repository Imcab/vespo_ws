"""
`vespo init`: sets a machine up from scratch, or re-checks one. Every
answer is saved as soon as it is given, so an interrupted init keeps them,
and the next run offers them as defaults.
"""

from typing import Annotated, Optional

import typer
import yaml

from .. import bashrc, completion, config, fx, procs, settings, system, ui
from . import load_config
from .build import colcon_command, run_colcon
from .doctor import ROSDEP_LIST
from .px4 import in_sync, sync_machine

# Asked after the build, in this order, so workspace models and worlds are offered
SIM_DEFAULTS = ("sim_model", "sim_world", "sim_pose", "sim_follow", "sim_headless")


def _save(cfg, setting, value):
    if value is not None:
        config.save_local_key(cfg, setting.key, value)
    return config.load(cfg.ws)


def _missing_repos(cfg):
    try:
        repos = (yaml.safe_load(cfg.repos_path.read_text()) or {}).get("repositories", {})
    except (OSError, yaml.YAMLError) as e:
        ui.abort(f"Cannot read {cfg.repos_path.name}: {e}")
    return [name for name in repos if not (cfg.ws / "src" / name / ".git").exists()]


def init(
    role: Annotated[Optional[str], typer.Option("--role", help="sim, companion or ground.", autocompletion=completion.roles)] = None,
    domain_id: Annotated[Optional[int], typer.Option("--domain-id", help="ROS_DOMAIN_ID.")] = None,
    flight_controller: Annotated[Optional[str], typer.Option("--flight-controller", help="Id from vespo.toml.", autocompletion=completion.flight_controllers)] = None,
    px4_path: Annotated[Optional[str], typer.Option("--px4-path", help="PX4-Autopilot checkout.")] = None,
    serial: Annotated[Optional[str], typer.Option("--serial", help="Serial device to the flight controller.", autocompletion=completion.serial_ports)] = None,
    baud: Annotated[Optional[int], typer.Option("--baud", help="Serial baud rate.")] = None,
    clone: Annotated[Optional[bool], typer.Option("--clone/--no-clone", help="Clone the pinned repos.")] = None,
    px4_sync: Annotated[Optional[bool], typer.Option("--px4-sync/--no-px4-sync", help="Put PX4 at the team version.")] = None,
    deps: Annotated[Optional[bool], typer.Option("--deps/--no-deps", help="rosdep install (uses sudo).")] = None,
    build: Annotated[Optional[bool], typer.Option("--build/--no-build", help="Build the workspace.")] = None,
    shell: Annotated[Optional[bool], typer.Option("--bashrc/--no-bashrc", help="Add the workspace to ~/.bashrc.")] = None,
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Accept defaults, ask nothing.")] = False,
):
    """Set this machine up: role, settings, repos, dependencies, build and ~/.bashrc."""
    cfg = load_config()
    fx.logo("Workspace setup")

    explicit = {"role": role, "domain_id": domain_id, "flight_controller": flight_controller,
                "px4_path": px4_path, "serial": serial, "baud": baud}

    # 1. Settings, role first: it decides which of the others apply. The
    # simulation defaults wait until after the build (step 5)
    ui.header("Machine settings")
    for setting in settings.SETTINGS.values():
        if setting.name in SIM_DEFAULTS:
            continue
        if setting.name != "role" and not setting.applies(cfg, cfg.role):
            continue
        cfg = _save(cfg, setting, setting.ask(cfg, explicit.get(setting.name), yes))
    ui.ok(f"Saved to {cfg.local_path.name}")

    # 2. Pinned third-party repos
    missing = _missing_repos(cfg)
    if missing:
        ui.header("Repositories")
        ui.info(f"Not cloned yet: {', '.join(missing)}")
        if settings.confirm("Clone them now with vcs?", clone, yes, True):
            (cfg.ws / "src").mkdir(exist_ok=True)
            if procs.run(["vcs", "import", "--input", str(cfg.repos_path), "src"], cwd=cfg.ws) != 0:
                ui.abort("vcs import failed")
            ui.ok("Repositories cloned")

    # 2b. PX4-Autopilot and px4_msgs at the team version. Without rebuilding:
    # the build of step 4 builds px4_msgs anyway
    px4 = cfg.get("px4.path")
    if px4 and system.is_px4_tree(px4) and (cfg.ws / "src" / "px4_msgs" / ".git").exists():
        ui.header("PX4")
        ref = cfg.get("px4.ref", "?")
        if in_sync(cfg):
            ui.ok(f"PX4-Autopilot and px4_msgs at the team version {ref}")
        elif settings.confirm(f"PX4 is not at the team version {ref}. Check it out now?", px4_sync, yes, True):
            sync_machine(cfg, rebuild=False)
        else:
            ui.warn("PX4 left as is. vespo doctor will keep reporting it; fix it with: vespo px4 sync")

    # 3. System dependencies of the packages in src/
    ui.header("Dependencies")
    if not ROSDEP_LIST.is_file():
        ui.warn("rosdep is not initialized, skipping. Fix: sudo rosdep init && rosdep update")
    elif settings.confirm("Install missing ROS dependencies with rosdep (asks for sudo)?", deps, yes, True):
        cmd = f"rosdep install --from-paths src --ignore-src -y -r --rosdistro {cfg.distro}"
        if procs.run(procs.ros_cmd(cfg.distro, cmd), cwd=cfg.ws) != 0:
            ui.warn("rosdep could not install everything; vespo build will show what is missing")

    # 4. Build
    ui.header("Build")
    if settings.confirm("Build the workspace now?", build, yes, True):
        if run_colcon(cfg, colcon_command(cfg, None, cfg.role)) != 0:
            ui.abort("Build failed, see the output above. Re-run with: vespo build")
        ui.ok("Workspace built")

    # 5. Simulation defaults. Their menus list the models and worlds that
    # workspace packages install, which only exist once the build has run
    sim_settings = [settings.SETTINGS[name] for name in SIM_DEFAULTS
                    if settings.SETTINGS[name].applies(cfg, cfg.role)]
    if sim_settings:
        ui.header("Simulation defaults")
        for setting in sim_settings:
            cfg = _save(cfg, setting, setting.ask(cfg, None, yes))
        ui.ok(f"Saved to {cfg.local_path.name}")

    # 6. Shell
    ui.header("Shell")
    if settings.confirm("Add vespo, the workspace and ROS_DOMAIN_ID to ~/.bashrc?", shell, yes, True):
        changed = bashrc.write(cfg)
        ui.ok("~/.bashrc updated (new terminals)" if changed else "~/.bashrc already up to date")
    completion = bashrc.install_completion()
    if completion is None:
        ui.warn("Could not install Tab completion for vespo")
    else:
        ui.ok("Tab completion installed (new terminals)" if completion else "Tab completion up to date")

    ui.console.print("")
    ui.ok(f"{cfg.role} machine ready")
    ui.info("Open a new terminal (or run: source ~/.bashrc), then check everything with: vespo doctor")
