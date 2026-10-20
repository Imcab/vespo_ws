"""Helpers to run external commands, either captured or attached to the terminal."""

import os
import shlex
import signal
import subprocess


def capture(cmd, cwd=None, timeout=15, env=None):
    """
    Runs a command and returns (exit_code, stdout). Never raises: a missing
    binary or a timeout returns code -1, so checks can treat both as failure.
    """
    try:
        proc = subprocess.run(
            cmd, cwd=cwd, env=_env(env), capture_output=True, text=True, timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired):
        return -1, ""
    return proc.returncode, proc.stdout.strip()


def run(cmd, cwd=None, env=None):
    """
    Runs a command attached to the terminal and returns its exit code.

    Ctrl+C in a terminal already reaches the child (same process group); it
    is forwarded anyway for when only vespo was signalled, e.g. by a script.
    Then the child gets time to shut down cleanly before being killed.
    """
    proc = subprocess.Popen(cmd, cwd=cwd, env=_env(env))
    try:
        return proc.wait()
    except KeyboardInterrupt:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        try:
            return proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            return proc.wait()


def ros_cmd(distro, script, ws=None):
    """
    Wraps a shell snippet so it runs with ROS sourced, whether or not the
    calling shell was. With `ws`, the workspace overlay is sourced too.
    """
    sources = [f"source /opt/ros/{shlex.quote(distro)}/setup.bash"]
    if ws is not None and (ws / "install" / "setup.bash").is_file():
        sources.append(f"source {shlex.quote(str(ws / 'install' / 'setup.bash'))}")
    return ["bash", "-c", " && ".join([*sources, script])]


def _env(extra):
    if not extra:
        return None
    return {**os.environ, **{k: str(v) for k, v in extra.items()}}
