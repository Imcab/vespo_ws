"""`vespo flash`: builds PX4 for the flight controller and uploads it over USB."""

from typing import Annotated, Optional

import typer

from .. import completion, procs, system, ui
from . import load_config, require_px4


def flash(
    no_upload: Annotated[bool, typer.Option("--no-upload", help="Only build the firmware.")] = False,
    target: Annotated[Optional[str], typer.Option("--target", "-t", help="PX4 make target, e.g. px4_fmu-v6c_default.", autocompletion=completion.targets)] = None,
):
    """Build PX4 for the configured flight controller and upload it."""
    cfg = load_config()
    px4 = require_px4(cfg)

    if not target:
        model = cfg.get("flight_controller.model")
        if not model:
            ui.abort("No flight controller set. Run: vespo config edit flight_controller (or pass --target)")
        board = cfg.get("flight_controllers", {}).get(model)
        if not board:
            ui.abort(f"Flight controller '{model}' is not listed in vespo.toml [flight_controllers]")
        target = board["target"]
        ui.header(f"Firmware for {board['name']}")
    else:
        ui.header(f"Firmware {target}")

    if not system.px4_board_file(px4, target).is_file():
        ui.abort(f"PX4 has no board for target {target} ({system.px4_board_file(px4, target)})")
    if not no_upload:
        if not system.in_group("dialout"):
            ui.warn("This session is not in the dialout group; the upload may not find the board")
        ui.info("Connect the flight controller over USB; the upload waits for it")

    goal = [target] if no_upload else [target, "upload"]
    code = procs.run(["make", *goal], cwd=px4)
    if code == 0:
        ui.ok(f"Firmware {'built' if no_upload else 'uploaded'}: {px4 / 'build' / target}")
    raise typer.Exit(code)
