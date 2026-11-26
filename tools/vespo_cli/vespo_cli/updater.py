"""
Keeps the CLI current without getting in the way.

Every command calls tick(). It only reads local git refs, says whether the
team pushed something and, at most every CHECK_EVERY seconds, starts a
detached `vespo update --background` that does the fetch. What that process
does next depends on the auto_update setting:

  cli     applies updates that only touch the CLI, tells about the rest
  notify  only tells
  off     nothing, not even the check

An update touching anything else needs a rebuild, so it waits for `vespo
update`. The CLI is installed editable, which is why a fast-forward is all it
takes to be running the new version on the next command.
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

from . import config, ui
from .procs import capture

CHECK_EVERY = 6 * 3600
FETCH_TIMEOUT = 60
STATE = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "vespo" / "update.json"
MODES = ("cli", "notify", "off")

CLI_DIR = "tools/vespo_cli/"
NEEDS_REINSTALL = "tools/vespo_cli/pyproject.toml"

# These run for a long time or must not change under the user: no background
# process is started from them. `update` does its own work.
BUSY_COMMANDS = {"agent", "build", "flash", "init", "simulate", "up", "update"}


def _git(ws, *args, timeout=15):
    return capture(["git", "-C", str(ws), *args], timeout=timeout, env={"GIT_TERMINAL_PROMPT": 0})


def upstream(ws):
    """Remote branch this checkout follows, e.g. 'origin/main', or None."""
    code, out = _git(ws, "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{upstream}")
    return out if code == 0 else None


def fetch(ws):
    return _git(ws, "fetch", "--quiet", "origin", timeout=FETCH_TIMEOUT)[0] == 0


def ahead_behind(ws, remote):
    """(local commits not pushed, remote commits not pulled)."""
    code, out = _git(ws, "rev-list", "--left-right", "--count", f"HEAD...{remote}")
    if code != 0:
        return 0, 0
    ahead, behind = out.split()
    return int(ahead), int(behind)


def changed_files(ws, old, new):
    return _git(ws, "diff", "--name-only", old, new)[1].splitlines()


def incoming(ws, remote):
    """One line per commit not pulled yet."""
    return _git(ws, "log", "--oneline", "--no-decorate", f"HEAD..{remote}")[1].splitlines()


def fast_forward(ws, remote):
    """False if it cannot be done cleanly, e.g. local edits to a file that changed."""
    return _git(ws, "merge", "--ff-only", "--quiet", remote, timeout=60)[0] == 0


def cli_only(files):
    """Whether these changes take effect with no rebuild or reinstall."""
    return bool(files) and all((f.startswith(CLI_DIR) and f != NEEDS_REINSTALL) or f.endswith(".md") for f in files)


def _read_state():
    try:
        return json.loads(STATE.read_text())
    except (OSError, ValueError):
        return {}


def _write_state(state):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(state))


def background(cfg):
    """Body of `vespo update --background`. Silent: nobody is watching it."""
    ws = cfg.ws
    remote = upstream(ws)
    if not remote or not fetch(ws):
        return
    ahead, behind = ahead_behind(ws, remote)
    if (cfg.get("update.auto", "cli") == "cli" and behind and not ahead
            and cli_only(changed_files(ws, "HEAD", remote)) and fast_forward(ws, remote)):
        state = _read_state()
        state["applied"] = state.get("applied", 0) + behind
        _write_state(state)


def _spawn(ws):
    subprocess.Popen(
        [sys.executable, "-m", "vespo_cli", "update", "--background"],
        cwd=ws, env={**os.environ, "VESPO_WS": str(ws)}, start_new_session=True,
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def _notify(ws, state):
    if state.pop("applied", None):
        ui.info("vespo updated itself to the team's latest version")
        _write_state(state)
    remote = upstream(ws)
    behind = ahead_behind(ws, remote)[1] if remote else 0
    if behind:
        ui.info(f"{behind} new commit(s) from the team. Get them with: vespo update")


def tick(command):
    """Called before every command. Must never make one fail or wait for the network."""
    if command == "update":
        return
    try:
        cfg = config.load()
        if cfg.get("update.auto", "cli") == "off":
            return
        state = _read_state()
        if sys.stdout.isatty():
            _notify(cfg.ws, state)
        if command not in BUSY_COMMANDS and time.time() - state.get("checked", 0) >= CHECK_EVERY:
            state["checked"] = time.time()
            _write_state(state)
            _spawn(cfg.ws)
    except Exception:
        pass
