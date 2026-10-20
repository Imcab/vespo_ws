"""Entry point: registers every subcommand on a single Typer app."""

from typing import Annotated

import typer
import typer.completion

from . import __version__, fx
from .commands import agent, build, clean, configure, doctor, flash, init, new, px4, qgc, simulate, up

# The group's --help is ours (the animated home screen), so Typer's default is
# turned off here and turned back on for each subcommand with HELP.
HELP = {"help_option_names": ["-h", "--help"]}

app = typer.Typer(
    name="vespo",
    add_completion=False,
    context_settings={"help_option_names": []},
)

# Tab completion stays on without Typer's --install-completion options in the
# help: this registers its shell classes, and `vespo init` installs the script.
typer.completion.completion_init()


def show_home(group_ctx):
    group = group_ctx.command
    entries = []
    for name in group.list_commands(group_ctx):
        cmd = group.get_command(group_ctx, name)
        if cmd is not None and not cmd.hidden:
            entries.append((name, cmd.get_short_help_str(limit=72)))
    # Typer lists subcommand groups (config) after plain commands; keep help last
    entries.sort(key=lambda entry: entry[0] == "help")
    fx.home(entries, f"VespoUAV workspace tool · v{__version__}")


@app.callback(invoke_without_command=True)
def main(
    ctx: typer.Context,
    show_help: Annotated[bool, typer.Option("--help", "-h", is_eager=True, help="Show this screen.")] = False,
):
    """VespoUAV workspace tool."""
    if show_help or ctx.invoked_subcommand is None:
        show_home(ctx)
        raise typer.Exit()


def help_(ctx: typer.Context):
    """Show this screen."""
    show_home(ctx.parent)


app.command(context_settings=HELP)(init.init)
app.command(context_settings=HELP)(doctor.doctor)
app.command(context_settings=HELP)(build.build)
app.command(context_settings=HELP)(new.new)
app.command(context_settings=HELP)(clean.clean)
app.command(context_settings=HELP)(agent.agent)
app.command(context_settings=HELP)(simulate.simulate)
app.command(context_settings=HELP)(flash.flash)
app.command(context_settings=HELP)(up.up)
app.command(context_settings=HELP)(up.down)
app.add_typer(configure.app, name="config")
app.add_typer(px4.app, name="px4")
app.add_typer(qgc.app, name="qgc")
app.command("help", context_settings=HELP)(help_)
