"""
Terminal effects: the logo, the animated home screen and the breathing
status line shown while `vespo doctor` works.

Everything degrades to plain static output when stdout is not an interactive
colour terminal (pipes, CI, TERM=dumb, NO_COLOR) or when VESPO_NO_ANIM is set.
"""

import math
import os
import random
import time
from contextlib import contextmanager

from rich.console import Group
from rich.live import Live
from rich.text import Text

from .ui import console

FPS = 30
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


def enabled():
    return (
        console.is_terminal
        and not console.is_dumb_terminal
        and console.color_system is not None
        and not os.environ.get("VESPO_NO_ANIM")
    )


# ---------------------------------------------------------------------------
# Colour helpers, all on (r, g, b) tuples
# ---------------------------------------------------------------------------

def _hex(colour):
    colour = colour.lstrip("#")
    return tuple(int(colour[i:i + 2], 16) for i in (0, 2, 4))


def _mix(a, b, t):
    return tuple(round(x + (y - x) * t) for x, y in zip(a, b))


def _css(rgb):
    return "#%02x%02x%02x" % rgb


def _gradient(stops, t):
    """Colour at position t (0..1) along a list of colour stops."""
    seg = max(0.0, min(1.0, t)) * (len(stops) - 1)
    i = min(int(seg), len(stops) - 2)
    return _mix(stops[i], stops[i + 1], seg - i)


def _smooth(t):
    return t * t * (3 - 2 * t)


def _ping_pong(pos):
    """0 -> 1 -> 0 -> 1 ... for an ever-growing pos, so gradients flow both ways."""
    pos %= 2.0
    return pos if pos <= 1 else 2 - pos


# ---------------------------------------------------------------------------
# Decode effect: characters start as random glyphs and resolve into the text
# ---------------------------------------------------------------------------

GLYPHS = "$S#@%&*sd¦/<>=+01▓▒░"
FLASH_SECONDS = 0.15  # a character glows briefly the moment it resolves


def _reveal_times(rows, start, span, jitter, seed):
    """
    When each character resolves: left to right over `span` seconds, with a
    fixed random jitter per character so the edge looks ragged, not a wipe.
    """
    rng = random.Random(seed)
    width = max(len(row) for row in rows)
    return [[start + span * x / width + rng.uniform(0, jitter) for x in range(len(row))] for row in rows]


def _decoded(ch, t, reveal, colour, glow=0.0):
    """(text, style) of one character at time t: a scrambled glyph or the real one."""
    if t < reveal:
        return random.choice(GLYPHS), _css(_mix(colour, BLACK, 0.55))
    flash = max(0.0, 1 - (t - reveal) / FLASH_SECONDS)
    return ch, f"bold {_css(_mix(colour, WHITE, max(glow * 0.85, flash * 0.7)))}"


# ---------------------------------------------------------------------------
# Logo
# ---------------------------------------------------------------------------

LOGO = r"""
sss$    ssss ,sS$"²²"$S▓, s@S$²²$S@s_ sS$$²""²$S@s ,sS$"²²"$S▓,
$$$$    $$$$ $$░$    $$▓$ $$$$   $$$$ $$$$    $$$$ $$░$    $$▓$
$$$$    $$$$ $$░$    $$$$ `"$$$$Ss,_  $$$$    $$$$ $$░$    $$$$
$$$$    $$$$ d$$$   $$$$$ ssss `"$$$$ $$$$    $$$$ $$$$    $$$$
$$$$    $$$$ $$$$S$$$$"'  $$$$   $$$$ $$$$    $$$$ $$$$    $$$$
▓$$$    $$$$ $$$$"'       $$$$   $$$$ $$$$    $$$$ ▓$$$    $$$$
▓$$$    $$$$ $$$$         $$$$   $$$$ $$$$    $$$$ ▓$$$    $$$$
▒$$$    $$$$ ▒$$$    $$$$ $$$$   $$$$ $$$$    $$$$ ▒$$$    $$$$
`²$$.  ,$$²' ░ $$    $$$$ $$$$   $$$$ $$$░bssd$$²' ░ $$    $$$$
  `²$ss$²'   `²$$bssd$$²' `²▓$bsd$$$' `²$░         `²$$bssd$$²'
""".strip("\n").splitlines()
LOGO_WIDTH = max(len(row) for row in LOGO)

# Red on top, through pale yellow, into orange at the bottom
LOGO_STOPS = [_hex(c) for c in ("#c81e1e", "#e8383d", "#f06a6a", "#fff3a0", "#f5d63d", "#e8932c", "#b8621b")]
LOGO_REVEAL = _reveal_times(LOGO, start=0.05, span=0.75, jitter=0.3, seed=7)
DECODE_SECONDS = 1.1  # every logo character has resolved by then


def _shine_at(t, start, seconds):
    """Column of the diagonal light band at time t, or None outside its sweep."""
    if t is None or not start <= t <= start + seconds:
        return None
    return -14 + (LOGO_WIDTH + 34) * (t - start) / seconds


def _logo_rows(t=None, shine=None):
    """
    The logo in its vertical gradient. With t, characters still decoding
    show as glyphs; `shine` is the column a slanted band of light crosses.
    t=None draws the finished logo. Narrow terminals get a wordmark.
    """
    if console.width < LOGO_WIDTH:
        word = Text()
        for i, ch in enumerate("V E S P O"):
            word.append(ch, style=f"bold {_css(_gradient(LOGO_STOPS, i / 8))}")
        return [word]

    rows = []
    for y, row in enumerate(LOGO):
        base = _gradient(LOGO_STOPS, y / (len(LOGO) - 1))
        if t is None:
            rows.append(Text(row, style=f"bold {_css(base)}"))
            continue
        text = Text()
        for x, ch in enumerate(row):
            if ch == " ":
                text.append(" ")
                continue
            glow = 0.0 if shine is None else max(0.0, 1 - abs(x + 2 * y - shine) / 6)
            text.append(*_decoded(ch, t, LOGO_REVEAL[y][x], base, glow))
        rows.append(text)
    return rows


def _play(frame, seconds):
    """Plays frame(t) for `seconds`, then leaves frame(None) on screen."""
    if not enabled():
        console.print(frame(None))
        return
    start = time.monotonic()
    with Live(frame(0.0), console=console, auto_refresh=False, transient=False) as live:
        try:
            while (t := time.monotonic() - start) < seconds:
                live.update(frame(t), refresh=True)
                time.sleep(1 / FPS)
        except KeyboardInterrupt:
            pass
        live.update(frame(None), refresh=True)


def logo(subtitle="", seconds=1.9):
    """The logo decoding itself, then one sweep of light across it."""
    def frame(t):
        parts = [Text(), *_logo_rows(t, _shine_at(t, DECODE_SECONDS - 0.2, seconds - DECODE_SECONDS + 0.2))]
        if subtitle:
            parts.append(Text(subtitle, style="dim"))
        return Group(*parts, Text())
    _play(frame, seconds)


# ---------------------------------------------------------------------------
# Home screen: bare `vespo`, `vespo help`, `vespo --help`
# ---------------------------------------------------------------------------

HOME_SECONDS = 2.3

# Command names flow through these, yellow to orange and back
WARM = [_hex(c) for c in ("#ffd23f", "#ffb020", "#ff9500", "#ff7518", "#ff5a1f")]


def _command_rows(entries, t, final=False):
    """
    One row per command, revealed one after another. Names decode letter by
    letter in a flowing yellow-orange gradient with a highlight sweeping down
    the list; help texts are dimmed. Hidden rows stay as blank lines so the
    screen height never jumps.
    """
    width = max(len(name) for name, _ in entries)
    rows = []
    for row, (name, help_) in enumerate(entries):
        start = 0.45 + row * 0.1
        if t < start:
            rows.append(Text())
            continue
        sweep = (t - 1.1) * 12 - row * 1.5  # letter the highlight is on in this row
        text = Text("  ")
        for i, ch in enumerate(name):
            colour = _gradient(WARM, _ping_pong(i * 0.12 + row * 0.1 + t * 0.35))
            glow = 0.0 if final else max(0.0, 1 - abs(i - sweep) / 2.5)
            text.append(*_decoded(ch, t, start + 0.05 + i * 0.03, colour, glow))
        text.append(" " * (width - len(name) + 4))
        room = console.width - len(text)
        if len(help_) > room:
            help_ = help_[:max(room - 1, 0)].rstrip() + "…"
        text.append(help_, style="dim")
        rows.append(text)
    return rows


def home(entries, subtitle):
    """entries: [(command name, one-line help)]."""
    def frame(t):
        final = t is None
        shine = _shine_at(t, DECODE_SECONDS - 0.2, 1.2)
        return Group(
            Text(),
            *_logo_rows(t, shine),
            Text(subtitle, style="dim"),
            Text(),
            Text("Commands", style="bold"),
            *_command_rows(entries, HOME_SECONDS if final else t, final),
            Text(),
            Text.assemble(("Run ", "dim"), "vespo <command> --help", (" for its options", "dim")),
            Text(),
        )
    _play(frame, HOME_SECONDS)


# ---------------------------------------------------------------------------
# Breathing status line
# ---------------------------------------------------------------------------

SPINNER = "·✢✳✶✻✽✻✶✳✢"
PULSE = [_hex(c) for c in ("#ff8c1a", "#ffb347", "#ff5e3a", "#ffd23f")]


class Pulse:
    """
    A spinning glyph and a label that breathes (fades dim and bright) while
    drifting through warm colours, with a glint running across the letters.
    Rich re-renders it on every refresh, so it animates on its own.
    """

    def __init__(self, label):
        self.label = label
        self.start = time.monotonic()

    def __rich__(self):
        t = time.monotonic() - self.start

        # Drift to the next palette colour every 1.5 s
        pos = (t / 1.5) % len(PULSE)
        i = int(pos)
        base = _mix(PULSE[i], PULSE[(i + 1) % len(PULSE)], _smooth(pos - i))

        # Breathe: brightness rises and falls every 1.6 s
        breath = 0.5 + 0.5 * math.sin(t * 2 * math.pi / 1.6)
        colour = _mix(_mix(base, BLACK, 0.45), base, breath)

        label = self.label + "…"
        glint = (t * 18) % (len(label) + 12) - 6

        text = Text(SPINNER[int(t * 10) % len(SPINNER)] + " ", style=f"bold {_css(base)}")
        for x, ch in enumerate(label):
            glow = max(0.0, 1 - abs(x - glint) / 4)
            text.append(ch, style=_css(_mix(colour, WHITE, glow * 0.6)))
        text.append(f"  ({int(t)}s)", style="dim")
        return text


class _Status:
    def __init__(self, pulse):
        self._pulse = pulse

    def update(self, label):
        if self._pulse:
            self._pulse.label = label

    def print(self, *renderables):
        # While a Live display is active, console output lands above it
        console.print(*renderables)


@contextmanager
def status(label):
    """
    Shows a Pulse at the bottom of the terminal while the block runs.
    `.update(label)` changes the text, `.print(...)` writes above it.
    """
    if not enabled():
        yield _Status(None)
        return
    pulse = Pulse(label)
    with Live(pulse, console=console, refresh_per_second=24, transient=True):
        yield _Status(pulse)
