"""`vespo clean`: deletes build output of some packages, or of the whole workspace."""

import shutil
from typing import Annotated, List, Optional

import typer

from .. import completion, ui
from . import load_config
from .build import colcon_command, run_colcon


def clean(
    packages: Annotated[Optional[List[str]], typer.Argument(help="Packages to clean.", autocompletion=completion.packages)] = None,
    all_: Annotated[bool, typer.Option("--all", help="Delete build/, install/ and log/ entirely.")] = False,
    rebuild: Annotated[bool, typer.Option("--rebuild", "-r", help="Build again afterwards.")] = False,
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Do not ask before --all.")] = False,
):
    """Delete build/ and install/ of packages (or everything with --all)."""
    cfg = load_config()
    if bool(packages) == all_:
        ui.abort("Name the packages to clean, or use --all")

    if all_:
        targets = [cfg.ws / name for name in ("build", "install", "log")]
        if not yes and not typer.confirm("Delete build/, install/ and log/ of the whole workspace?"):
            raise typer.Exit(1)
    else:
        targets = [cfg.ws / folder / pkg for pkg in packages for folder in ("build", "install")]

    for path in targets:
        if path.exists():
            shutil.rmtree(path)
            ui.ok(f"Deleted {path.relative_to(cfg.ws)}")
        else:
            ui.status("skip", f"{path.relative_to(cfg.ws)} does not exist")

    if rebuild:
        code = run_colcon(cfg, colcon_command(cfg, None if all_ else packages, cfg.role))
        raise typer.Exit(code)
