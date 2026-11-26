"""
`vespo update`: brings this machine to the team's latest version.

Pulls the workspace, then redoes only what the pull made necessary. That second
half runs as `vespo update --finish OLD_COMMIT` in a new process, so it is the
code just pulled that decides what to do, not the version being replaced.
"""

import os
import sys
from typing import Annotated, Optional

import typer

from .. import bashrc, procs, system, ui, updater
from . import load_config
from .build import colcon_command, rosdep_install, run_colcon
from .px4 import sync_machine

SHOWN_COMMITS = 15


def _reinstall_cli(cfg):
    ui.info("Reinstalling the vespo CLI, its dependencies changed")
    code = procs.run(["python3", "-m", "pip", "--python", sys.executable, "install", "--quiet",
                      "--disable-pip-version-check", "-e", str(cfg.ws / "tools" / "vespo_cli")])
    if code != 0:
        ui.abort("Could not reinstall the CLI. Run: ./bootstrap.sh")


def _finish(cfg, old, build):
    files = updater.changed_files(cfg.ws, old, "HEAD")

    if updater.NEEDS_REINSTALL in files:
        _reinstall_cli(cfg)
    if any(f.startswith(updater.CLI_DIR) for f in files):
        bashrc.install_completion()

    repos_changed = cfg.repos_path.name in files
    if repos_changed:
        ui.info("Updating the pinned repositories")
        if procs.run(["vcs", "import", "--input", str(cfg.repos_path), "src"], cwd=cfg.ws) != 0:
            ui.abort("vcs import failed. Check the internet connection and run: vcs import src < vespo.repos")

    px4 = cfg.get("px4.path")
    if (repos_changed or cfg.project_path.name in files) and px4 and system.is_px4_tree(px4):
        sync_machine(cfg, rebuild=False)

    if any(f.endswith("package.xml") for f in files):
        rosdep_install(cfg)

    if repos_changed or any(f.startswith("src/") for f in files):
        if not build:
            ui.warn("Code changed and the build was skipped. Build it with: vespo build")
        elif run_colcon(cfg, colcon_command(cfg, None, cfg.role)) != 0:
            ui.abort("Build failed, see the output above. Re-run with: vespo build")
        else:
            ui.ok("Workspace built. New terminals pick it up; in this one run: source install/setup.bash")

    ui.ok(f"Up to date at {system.git_head(cfg.ws)[:10]}")


def update(
    check: Annotated[bool, typer.Option("--check", help="Only show what is new.")] = False,
    build: Annotated[bool, typer.Option("--build/--no-build", help="Rebuild if the code changed.")] = True,
    background: Annotated[bool, typer.Option("--background", hidden=True)] = False,
    finish: Annotated[Optional[str], typer.Option("--finish", hidden=True)] = None,
):
    """Get the team's latest version: pull, then rebuild only what changed."""
    cfg = load_config()
    if background:
        updater.background(cfg)
        return
    if finish:
        _finish(cfg, finish, build)
        return

    ui.header("Updating")
    remote = updater.upstream(cfg.ws)
    if not remote:
        ui.abort("This checkout follows no remote branch. Go back to it with: git checkout main")
    if not updater.fetch(cfg.ws):
        ui.abort("git fetch failed. Check the internet connection")

    ahead, behind = updater.ahead_behind(cfg.ws, remote)
    if not behind:
        ui.ok(f"Already up to date at {system.git_head(cfg.ws)[:10]}")
        return

    commits = updater.incoming(cfg.ws, remote)
    for line in commits[:SHOWN_COMMITS]:
        ui.info(line)
    if len(commits) > SHOWN_COMMITS:
        ui.info(f"... and {len(commits) - SHOWN_COMMITS} more")
    if check:
        ui.info("Get them with: vespo update")
        return

    if ahead:
        ui.abort(f"You have {ahead} commit(s) not pushed. Run: git pull --rebase, then vespo update")
    old = system.git_head(cfg.ws)
    if not updater.fast_forward(cfg.ws, remote):
        ui.abort("Your local changes touch the same files. Commit or stash them, then run: vespo update")
    ui.ok(f"Pulled {behind} commit(s)")

    args = [sys.executable, "-m", "vespo_cli", "update", "--finish", old]
    if not build:
        args.append("--no-build")
    os.execv(sys.executable, args)
