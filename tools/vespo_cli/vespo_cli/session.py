"""
The processes `vespo up` starts and `vespo down` stops.

Inside kitty (with remote control on) each process gets its own pane in a new
"vespo" tab and up returns at once. Elsewhere, e.g. over SSH, up runs them all
in the foreground with each line of output prefixed by the process name, and
Ctrl+C stops them all.

Every process runs in its own process group, recorded in a state file, so
`vespo down` can stop the whole tree (make -> px4, gz sim server and GUI...)
from any terminal.
"""

import json
import os
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from rich.markup import escape
from rich.text import Text

from . import procs, ui

STATE = Path.home() / ".cache" / "vespo" / "up.json"
TAB_TITLE = "vespo"
COLOURS = ["cyan", "magenta", "green", "yellow", "blue"]

# Exit codes of a process stopped by Ctrl+C: killed by SIGINT, the shell's
# 130, or a vespo command passing on its child's -2 (exit status 254)
CLEAN_EXIT = (0, -signal.SIGINT, 130, 254)

# Leftovers `vespo down` cleans even when the process that started them is
# gone: PX4 SITL, Gazebo (server and GUI, a ruby script) and the agent. Each
# is a pgrep pattern anchored on the executable, so a shell or editor that
# merely mentions them is left alone. Only this user's processes.
STRAYS = {
    "sim": [r"^\S*px4_sitl_default/bin/px4( |$)", r"^(\S*ruby\S* )?\S*gz sim( |$)"],
    "agent": [r"^\S*MicroXRCEAgent( |$)"],
}

# A kitty pane runs its command through this, so it stays open with the log
# after the command exits until Enter. $1 is the name shown, the rest the command.
HOLD = ('name="$1"; shift; "$@"; code=$?; '
        'printf "\\n%s exited (code %s). Enter closes this pane.\\n" "$name" "$code"; read -r _')


@dataclass
class Service:
    name: str
    cmd: list
    cwd: Path
    label: str           # what the user would type to run it alone
    env: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

def load_state():
    try:
        return json.loads(STATE.read_text())
    except (OSError, ValueError):
        return None


def save_state(state):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(state, indent=2))


def clear_state():
    STATE.unlink(missing_ok=True)


def running(state):
    """Names of the recorded processes still alive."""
    return [p["name"] for p in (state or {}).get("procs", []) if _group_left(p)]


# ---------------------------------------------------------------------------
# Kitty
# ---------------------------------------------------------------------------

def kitty_to():
    """`--to` argument for kitty @ when kitty listens on a socket, else []."""
    socket = os.environ.get("KITTY_LISTEN_ON")
    return ["--to", socket] if socket else []


def kitty_available():
    """(True, None) inside kitty with remote control allowed, else (False, why or None)."""
    if not os.environ.get("KITTY_WINDOW_ID"):
        return False, None
    if not shutil.which("kitty"):
        return False, "kitty is not on PATH"
    code, _ = procs.capture(["kitty", "@", *kitty_to(), "ls", "--match", f"id:{os.environ['KITTY_WINDOW_ID']}"],
                            timeout=5)
    if code != 0:
        return False, "kitty remote control is off. Add 'allow_remote_control yes' to kitty.conf and restart kitty"
    return True, None


def _kitty(*args, to=None):
    return procs.capture(["kitty", "@", *(to if to is not None else kitty_to()), *args], timeout=10)


def _kitty_pid(window):
    code, out = _kitty("ls", "--match", f"id:{window}")
    if code != 0:
        return None
    for os_window in json.loads(out):
        for tab in os_window["tabs"]:
            for win in tab["windows"]:
                if win["id"] == window:
                    return win["pid"]
    return None


def start_kitty(services, extra_env):
    """
    Opens a "vespo" tab with one pane per service: the first on the left, the
    rest stacked on the right. Returns the state to save.
    """
    started, first, previous = [], None, None
    for i, service in enumerate(services):
        cmd = ["launch", "--title", service.name, "--cwd", str(service.cwd), "--copy-env"]
        for key, value in {**extra_env, **service.env}.items():
            cmd += ["--env", f"{key}={value}"]
        if first is None:
            cmd += ["--type=tab", "--tab-title", TAB_TITLE]
        else:
            location = "vsplit" if i == 1 else "hsplit"
            cmd += ["--type=window", "--keep-focus", "--next-to", f"id:{previous}", f"--location={location}"]
        cmd += ["bash", "-c", HOLD, "bash", service.name, *service.cmd]

        code, out = _kitty(*cmd)
        if code != 0 or not out.strip().isdigit():
            ui.fail(f"kitty could not open the {service.name} pane")
            stop({"kitty": first and {"window": first, "to": kitty_to()}, "procs": started})
            return None
        window = int(out.strip())
        first = first or window
        previous = window
        pid = _kitty_pid(window)
        if pid:
            # The pane's bash is the group leader and waits at `read` once its
            # command ends: only the rest of the group counts as running
            started.append({"name": service.name, "pid": pid, "holder": True})
        ui.ok(f"{service.name}: {service.label}")
    return {"kitty": {"window": first, "to": kitty_to()}, "procs": started}


# ---------------------------------------------------------------------------
# Foreground, one terminal
# ---------------------------------------------------------------------------

def start_foreground(services, extra_env, on_started):
    """
    Runs every service with its output prefixed by its name until all exit or
    Ctrl+C (or SIGTERM/SIGHUP, e.g. a closed SSH session), which stops all.
    on_started(state) is called once they are running, to save the state.
    """
    width = max(len(s.name) for s in services)
    env = {**os.environ, **{k: str(v) for k, v in extra_env.items()}, "PYTHONUNBUFFERED": "1",
           "FORCE_COLOR": "1"}
    running_procs, readers = [], []
    for i, service in enumerate(services):
        ui.ok(f"{service.name}: {service.label}")
        # stdin is a pipe nobody writes: the PX4 shell spins on EOF but
        # simply waits on an open pipe
        proc = subprocess.Popen(service.cmd, cwd=service.cwd, env={**env, **{k: str(v) for k, v in service.env.items()}},
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                start_new_session=True, text=True, errors="replace", bufsize=1)
        colour = COLOURS[i % len(COLOURS)]
        prefix = f"[{colour}]{escape(service.name.ljust(width))}[/] │ "
        reader = threading.Thread(target=_relay, args=(proc, prefix), daemon=True)
        reader.start()
        running_procs.append((service, proc))
        readers.append(reader)

    state = {"kitty": None, "procs": [{"name": s.name, "pid": p.pid, "holder": False} for s, p in running_procs]}
    on_started(state)
    ui.info("Ctrl+C stops everything")

    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGTERM, signal.SIGHUP)}
    reported = set()
    try:
        while True:
            for service, proc in running_procs:
                if proc.poll() is not None and service.name not in reported:
                    reported.add(service.name)
                    (ui.info if proc.returncode in CLEAN_EXIT else ui.warn)(
                        f"{service.name} exited (code {proc.returncode})")
            if len(reported) == len(running_procs):
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        ui.console.print()
        stop(state)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    for reader in readers:
        reader.join(timeout=2)
    return state


def _relay(proc, prefix):
    for line in proc.stdout:
        # Keep the agent's and Gazebo's own colours instead of printing their escape codes
        text = Text.from_markup(prefix)
        text.append_text(Text.from_ansi(line.rstrip("\n")))
        ui.console.print(text, highlight=False, soft_wrap=True)


# ---------------------------------------------------------------------------
# Stopping
# ---------------------------------------------------------------------------

def _group_left(entry):
    """Pids of the entry's process group still alive (minus a holding pane shell)."""
    pid = entry["pid"]
    try:
        os.waitpid(pid, os.WNOHANG)   # reap it if it is our child
    except ChildProcessError:
        pass
    _, out = procs.capture(["pgrep", "-g", str(pid)], timeout=5)
    left = {int(p) for p in out.split()}
    if entry.get("holder"):
        left.discard(pid)
    return left


def _signal_group(entry, sig):
    for pid in _group_left(entry):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


def stop(state, grace=15):
    """
    Stops the recorded processes: SIGINT (what Ctrl+C sends, so PX4, Gazebo
    and ROS shut down cleanly), then SIGTERM, then SIGKILL. Closes the kitty
    tab. Returns the names that had to be killed.
    """
    entries = [e for e in (state or {}).get("procs", []) if _group_left(e)]
    forced = []
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        if not entries:
            break
        if sig != signal.SIGINT:
            forced += [e["name"] for e in entries if e["name"] not in forced]
        for entry in entries:
            _signal_group(entry, sig)
        deadline = time.monotonic() + wait
        while entries and time.monotonic() < deadline:
            time.sleep(0.3)
            entries = [e for e in entries if _group_left(e)]

    kitty = (state or {}).get("kitty")
    if kitty and kitty.get("window"):
        _kitty("close-tab", "--ignore-no-match", "--match", f"window_id:{kitty['window']}", to=kitty.get("to") or [])
    return forced


def kill_strays(kinds):
    """Terminates leftover PX4 / Gazebo / agent processes of this user. Returns their command lines."""
    found = {}
    for kind in kinds:
        for pattern in STRAYS[kind]:
            _, out = procs.capture(["pgrep", "-u", str(os.getuid()), "-a", "-f", pattern], timeout=5)
            for line in out.splitlines():
                pid, _, cmdline = line.partition(" ")
                if int(pid) != os.getpid():
                    found[int(pid)] = cmdline
    alive = list(found)
    for sig, wait in ((signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        for pid in alive:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + wait
        while alive and time.monotonic() < deadline:
            time.sleep(0.3)
            alive = [pid for pid in alive if _exists(pid)]
    return list(found.values())


def _exists(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True
