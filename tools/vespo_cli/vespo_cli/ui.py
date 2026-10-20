"""Terminal output: one console and consistent status lines for every command."""

import typer
from rich.console import Console
from rich.markup import escape

console = Console()

# status -> (icon, colour)
STYLES = {
    "ok": ("✔", "green"),
    "warn": ("!", "yellow"),
    "fail": ("✘", "red"),
    "skip": ("-", "dim"),
    "info": ("•", "cyan"),
}


def header(title):
    console.rule(f"[bold]{escape(title)}", align="left")


def status(kind, message):
    icon, colour = STYLES[kind]
    console.print(f"[{colour}]{icon}[/] {escape(message)}")


def ok(message):
    status("ok", message)


def info(message):
    status("info", message)


def warn(message):
    status("warn", message)


def fail(message):
    status("fail", message)


def abort(message, code=1):
    fail(message)
    raise typer.Exit(code)
