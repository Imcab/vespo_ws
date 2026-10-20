"""`vespo config`: show, set or re-ask single machine settings without a full init."""

from typing import Annotated, Optional

import typer
from rich.table import Table

from .. import bashrc, completion, config, settings, ui
from . import load_config

app = typer.Typer(
    help="Show or change this machine's settings.",
    context_settings={"help_option_names": ["-h", "--help"]},
)


def _source(cfg, key):
    """Where a value comes from: this machine, the project or nowhere."""
    node = cfg.local
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            return "project" if cfg.get(key) is not None else ""
        node = node[part]
    return "local"


@app.callback(invoke_without_command=True)
def show(ctx: typer.Context):
    """Show every setting and where its value comes from."""
    if ctx.invoked_subcommand is not None:
        return
    cfg = load_config()
    table = Table(title=f"Machine settings · {cfg.ws}", title_justify="left")
    table.add_column("Setting", style="cyan")
    table.add_column("Value")
    table.add_column("From", style="dim")
    for setting in settings.SETTINGS.values():
        used = setting.applies(cfg, cfg.role)
        value = setting.describe(cfg)
        table.add_row(setting.name, value if used else f"[dim]{value} (unused by role)[/]", _source(cfg, setting.key))
    ui.console.print(table)
    ui.info("Change one with: vespo config set NAME VALUE  or  vespo config edit NAME")


def _apply(cfg, setting, value):
    if value is None:   # e.g. "None yet" in the flight controller menu
        local = {section: dict(values) for section, values in cfg.local.items()}
        section, _, key = setting.key.partition(".")
        local.get(section, {}).pop(key, None)
        config.write_local(cfg.ws, local)
        ui.ok(f"{setting.name} unset")
        return
    config.save_local_key(cfg, setting.key, value)
    ui.ok(f"{setting.name} = {value}")
    cfg = config.load(cfg.ws)
    if setting.name == "domain_id" and bashrc.installed():
        bashrc.write(cfg)
        ui.ok("Updated ROS_DOMAIN_ID in ~/.bashrc (new terminals)")
    if setting.name == "role":
        for missing in settings.missing_for_role(cfg, value):
            ui.info(f"The {value} role also needs {missing.name}")
            answer = missing.ask(cfg, None, False)
            if answer is not None:
                config.save_local_key(cfg, missing.key, answer)
                cfg = config.load(cfg.ws)


@app.command("set")
def set_(
    name: Annotated[str, typer.Argument(help="Setting name, see: vespo config", autocompletion=completion.setting_names)],
    value: Annotated[str, typer.Argument(help="New value", autocompletion=completion.setting_values)],
):
    """Change one setting, validated."""
    cfg = load_config()
    setting = settings.SETTINGS.get(name)
    if not setting:
        ui.abort(f"Unknown setting '{name}'. Settings: {', '.join(settings.SETTINGS)}")
    try:
        parsed = setting.parse(cfg, value)
    except ValueError as e:
        ui.abort(str(e))
    _apply(cfg, setting, parsed)


@app.command()
def edit(name: Annotated[Optional[str], typer.Argument(help="Setting name; omit for a menu.", autocompletion=completion.setting_names)] = None):
    """Ask one setting again from a menu."""
    cfg = load_config()
    if name is None:
        options = [(s.name, s.label) for s in settings.SETTINGS.values() if s.applies(cfg, cfg.role)]
        name = settings.choose("Setting to change", options, None, None, False, "NAME")
    setting = settings.SETTINGS.get(name)
    if not setting:
        ui.abort(f"Unknown setting '{name}'. Settings: {', '.join(settings.SETTINGS)}")
    _apply(cfg, setting, setting.ask(cfg, None, False))
