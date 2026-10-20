"""Allows `python3 -m vespo_cli`, used by bootstrap.sh before ~/.local/bin is on PATH."""

from .main import app

app(prog_name="vespo")
