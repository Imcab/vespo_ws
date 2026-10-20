"""
`vespo qgc`: QGroundControl, the ground station used to set up and fly PX4.

It ships as one AppImage per architecture. The installed version is read
from the AppImage itself (X-AppImage-Version in its .desktop file), which
works without FUSE, and compared with the latest GitHub release.
"""

import json
import os
import platform
import re
import shutil
import subprocess
import tempfile
import urllib.request
from pathlib import Path
from typing import Annotated, Optional

import typer
from rich.progress import BarColumn, DownloadColumn, Progress, TransferSpeedColumn

from .. import config, procs, system, ui
from . import load_config

app = typer.Typer(
    help="Launch, check or install QGroundControl.",
    context_settings={"help_option_names": ["-h", "--help"]},
)

RELEASES = "https://api.github.com/repos/mavlink/qgroundcontrol/releases"
LOG = Path.home() / ".cache" / "vespo" / "qgc.log"

# apt packages the AppImage needs on Ubuntu, per the QGroundControl docs
REQUIRED_PKGS = {
    "libfuse2t64": "runs AppImages",
    "libxcb-cursor0": "Qt windows",
    "gstreamer1.0-plugins-bad": "video",
    "gstreamer1.0-libav": "video",
    "gstreamer1.0-gl": "video",
}


def asset_name():
    arch = {"x86_64": "x86_64", "amd64": "x86_64", "aarch64": "aarch64", "arm64": "aarch64"}.get(platform.machine())
    return f"QGroundControl-{arch}.AppImage" if arch else None


def default_path(cfg):
    return cfg.ws.parent / "apps" / (asset_name() or "QGroundControl.AppImage")


def candidates(cfg):
    """Existing QGroundControl AppImages in the usual places, best guess first."""
    places = [cfg.ws.parent / "apps", Path.home() / "Applications", Path.home() / "Downloads", Path.home()]
    found = []
    for place in places:
        found += sorted(place.glob("QGroundControl*.AppImage"))
    on_path = shutil.which("QGroundControl")
    if on_path:
        found.append(Path(on_path))
    return list(dict.fromkeys(p.resolve() for p in found if p.is_file()))


def find(cfg):
    """The configured AppImage if it exists, else the first one found, else None."""
    configured = cfg.get("qgc.path")
    if configured and Path(configured).is_file():
        return Path(configured)
    found = candidates(cfg)
    return found[0] if found else None


def installed_version(path):
    """Version string of an AppImage, e.g. '5.1.5', or None."""
    with tempfile.TemporaryDirectory() as tmp:
        code, _ = procs.capture([str(path), "--appimage-extract", "*.desktop"], cwd=tmp, timeout=60)
        if code != 0:
            return None
        for desktop in Path(tmp, "squashfs-root").glob("*.desktop"):
            match = re.search(r"^X-AppImage-Version=(.+)$", desktop.read_text(errors="ignore"), re.M)
            if match:
                return match.group(1).strip()
    return None


def latest_release():
    """(version, download url) of the newest stable release for this machine, or (None, None)."""
    try:
        with urllib.request.urlopen(f"{RELEASES}/latest", timeout=15) as response:
            data = json.load(response)
    except (OSError, ValueError):
        return None, None
    url = next((a["browser_download_url"] for a in data.get("assets", []) if a["name"] == asset_name()), None)
    return data.get("tag_name", "").lstrip("v") or None, url


def _version_key(version):
    return tuple(int(n) for n in re.findall(r"\d+", version or "0"))


def package_installed(pkg):
    # dpkg -s also succeeds for removed packages that left config files ("rc")
    _, out = procs.capture(["dpkg-query", "-W", "-f=${Status}", pkg])
    return out == "install ok installed"


def missing_packages():
    return [pkg for pkg in REQUIRED_PKGS if not package_installed(pkg)]


def modemmanager_installed():
    return package_installed("modemmanager")


def running():
    code, _ = procs.capture(["pgrep", "-x", "QGroundControl"])
    return code == 0


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def check(cfg):
    """
    The AppImage to run, after warning about what may break it, or None
    (reported) when it cannot start.
    """
    path = find(cfg)
    if not path:
        ui.fail("QGroundControl is not installed. Fix: vespo qgc install")
        return None
    missing = missing_packages()
    if "libfuse2t64" in missing:
        ui.fail("QGroundControl needs libfuse2t64 to start. Fix: sudo apt install libfuse2t64")
        return None
    if missing:
        ui.warn(f"Missing {', '.join(missing)}; some features may not work. See: vespo qgc status")
    if modemmanager_installed():
        ui.warn("ModemManager is installed and can block the USB link to the flight controller. "
                "Fix: sudo apt remove modemmanager")
    return path


def start(path):
    """Starts QGroundControl detached from this terminal. Returns its pid."""
    LOG.parent.mkdir(parents=True, exist_ok=True)
    with open(LOG, "w") as log:
        proc = subprocess.Popen([str(path)], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
    ui.ok(f"QGroundControl started (log: {LOG})")
    return proc.pid


@app.callback(invoke_without_command=True)
def launch(
    ctx: typer.Context,
    foreground: Annotated[bool, typer.Option("--foreground", "-f", help="Keep it attached and show its log.")] = False,
):
    """Launch QGroundControl."""
    if ctx.invoked_subcommand is not None:
        return
    cfg = load_config()
    if running():
        ui.ok("QGroundControl is already running")
        return
    path = check(cfg)
    if not path:
        raise typer.Exit(1)
    if foreground:
        ui.header(f"QGroundControl · {path}")
        raise typer.Exit(procs.run([str(path)]))
    start(path)


@app.command()
def status():
    """Show where QGroundControl is, its version and whether this machine is ready for it."""
    cfg = load_config()
    ui.header("QGroundControl")
    path = find(cfg)
    latest, _ = latest_release()

    if not path:
        ui.fail("Not installed")
        ui.info(f"Latest release: {latest or 'unknown (offline?)'}. Install: vespo qgc install")
    else:
        version = installed_version(path)
        ui.ok(f"Installed: {version or 'unknown version'} at {path}")
        if not cfg.get("qgc.path"):
            ui.info(f"Not saved in the settings yet. Fix: vespo config set qgc_path {path}")
        if latest is None:
            ui.warn("Could not check the latest release (offline?)")
        elif version and _version_key(latest) > _version_key(version):
            ui.warn(f"Version {latest} is available. Update: vespo qgc install")
        else:
            ui.ok(f"Up to date (latest release {latest})")
        if running():
            ui.info("Running now")

    for pkg in missing_packages():
        ui.fail(f"{pkg} missing ({REQUIRED_PKGS[pkg]}). Fix: sudo apt install {pkg}")
    if not missing_packages():
        ui.ok("System packages present")
    if modemmanager_installed():
        ui.fail("ModemManager installed, it can grab the flight controller's USB port. Fix: sudo apt remove modemmanager")
    else:
        ui.ok("ModemManager not installed")
    if system.in_group("dialout"):
        ui.ok("Session in the dialout group (USB to the flight controller)")
    else:
        ui.fail("Session not in the dialout group. Fix: sudo usermod -aG dialout $USER, then log out and back in")


@app.command()
def install(
    path: Annotated[Optional[Path], typer.Option("--path", help="Where to put the AppImage.")] = None,
    force: Annotated[bool, typer.Option("--force", help="Download even if already up to date.")] = False,
):
    """Download the latest QGroundControl, or update the installed one."""
    cfg = load_config()
    if not asset_name():
        ui.abort(f"QGroundControl has no AppImage for {platform.machine()}")

    target = (path.expanduser().resolve() if path else find(cfg) or default_path(cfg))
    latest, url = latest_release()
    if not url:
        ui.abort("Could not find the latest release on GitHub (offline?)")

    ui.header(f"QGroundControl {latest}")
    if target.is_file() and not force:
        current = installed_version(target)
        if current and _version_key(current) >= _version_key(latest):
            ui.ok(f"Already up to date: {current} at {target}")
            _remember(cfg, target)
            return
        ui.info(f"Updating {current or 'unknown version'} -> {latest}")
    if running():
        ui.abort("Close QGroundControl first, then run this again")

    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + ".part")
    try:
        _download(url, partial)
    except (OSError, KeyboardInterrupt) as e:
        partial.unlink(missing_ok=True)
        ui.abort(f"Download failed: {e}" if isinstance(e, OSError) else "Download cancelled")
    partial.chmod(0o755)
    os.replace(partial, target)
    ui.ok(f"Installed {latest} at {target}")
    _remember(cfg, target)

    for pkg in missing_packages():
        ui.warn(f"{pkg} missing ({REQUIRED_PKGS[pkg]}). Fix: sudo apt install {pkg}")
    ui.info("Launch it with: vespo qgc")


def _remember(cfg, target):
    if cfg.get("qgc.path") != str(target):
        config.save_local_key(cfg, "qgc.path", str(target))
        ui.ok("Saved as qgc_path in vespo.local.toml")


def _download(url, dest):
    with urllib.request.urlopen(url, timeout=30) as response, open(dest, "wb") as out:
        total = int(response.headers.get("Content-Length", 0)) or None
        with Progress("[progress.description]{task.description}", BarColumn(), DownloadColumn(),
                      TransferSpeedColumn(), console=ui.console, transient=True) as progress:
            task = progress.add_task("Downloading", total=total)
            while chunk := response.read(1 << 20):
                out.write(chunk)
                progress.advance(task, len(chunk))
