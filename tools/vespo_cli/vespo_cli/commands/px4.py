"""
`vespo px4`: which PX4 this workspace is pinned to, and moving that pin.

Two different jobs:
  sync    puts THIS machine at the versions pinned in vespo.toml / vespo.repos.
          It never changes the pin. This is what a teammate runs after cloning
          or pulling.
  update  moves the pin of the WHOLE TEAM to another PX4 version: checks PX4
          out, finds the px4_msgs commit with exactly the same messages
          (msgs.find_matching_commit) and writes both to vespo.toml /
          vespo.repos, to be committed.
Both rebuild px4_msgs and everything that depends on it.
"""

import re
from typing import Annotated, Optional

import typer
import yaml

from .. import completion, msgs, procs, settings, system, ui
from . import load_config, require_px4
from .build import run_colcon

app = typer.Typer(
    help="Show or change the PX4 version of the workspace.",
    context_settings={"help_option_names": ["-h", "--help"]},
)

STABLE_TAG = re.compile(r"^v(\d+)\.(\d+)\.(\d+)$")
PINNED = "pinned"


def _msgs_repo(cfg):
    path = cfg.ws / "src" / "px4_msgs"
    if not (path / ".git").exists():
        ui.abort("src/px4_msgs is missing. Run: vespo init (or: vcs import src < vespo.repos)")
    return path


def _pinned_msgs(cfg):
    """px4_msgs version pinned in vespo.repos, or None."""
    try:
        repos = (yaml.safe_load(cfg.repos_path.read_text()) or {}).get("repositories", {})
    except (OSError, yaml.YAMLError) as e:
        ui.abort(f"Cannot read {cfg.repos_path.name}: {e}")
    version = (repos.get("px4_msgs") or {}).get("version")
    return str(version) if version else None


def _describe(repo, commit):
    _, out = procs.capture(["git", "-C", str(repo), "describe", "--tags", "--always", commit])
    return out or commit[:10]


def latest_stable(px4):
    _, out = procs.capture(["git", "-C", str(px4), "tag", "-l", "v*"])
    tags = [t for t in out.split() if STABLE_TAG.match(t)]
    return max(tags, key=lambda t: tuple(int(n) for n in STABLE_TAG.match(t).groups()), default=None)


@app.callback(invoke_without_command=True)
def status(ctx: typer.Context):
    """Show the pinned PX4 version and whether PX4 and px4_msgs match it."""
    if ctx.invoked_subcommand is not None:
        return
    cfg = load_config()
    px4 = require_px4(cfg)
    pinned = cfg.get("px4.commit")
    head = system.git_head(px4)
    ui.header("PX4")
    ui.info(f"Pinned: {cfg.get('px4.ref', '?')} ({pinned[:10]})")
    if head == pinned:
        ui.ok(f"PX4-Autopilot at {_describe(px4, head)}")
    else:
        ui.warn(f"PX4-Autopilot at {_describe(px4, head)}, not the pinned commit. Fix: vespo px4 sync")

    repo = cfg.ws / "src" / "px4_msgs"
    if not (repo / ".git").exists():
        ui.fail("src/px4_msgs is missing. Run: vespo init")
        return
    missing, extra, changed = msgs.diff(msgs.message_blobs(px4, head) or {},
                                        msgs.message_blobs(repo, system.git_head(repo)) or {})
    if not (missing or extra or changed):
        ui.ok("px4_msgs carries exactly the same messages as PX4")
    else:
        ui.fail(f"px4_msgs differs from PX4: {len(missing)} missing, {len(extra)} extra, {len(changed)} changed")
        ui.info("Fix: vespo px4 sync")


def _git(repo, *args, timeout=600):
    code, _ = procs.capture(["git", "-C", str(repo), *args], timeout=timeout)
    return code == 0


def _checkout(px4, commit, repo, msgs_commit):
    """Checks PX4 (with its submodules) and px4_msgs out at the given commits."""
    code, dirty = procs.capture(["git", "-C", str(px4), "status", "--porcelain", "--untracked-files=no"])
    if dirty:
        ui.abort("PX4-Autopilot has uncommitted changes. Commit or stash them first")

    if system.git_head(px4) != commit:
        if not _git(px4, "checkout", "--quiet", "--detach", commit):
            ui.abort("Could not check out PX4-Autopilot")
    ui.info("Updating PX4 submodules (this can take a few minutes)")
    if not (_git(px4, "submodule", "sync", "--recursive")
            and _git(px4, "submodule", "update", "--init", "--recursive", "--jobs", "8", timeout=1800)):
        ui.abort("Updating the PX4 submodules failed. Run in PX4-Autopilot: git submodule update --init --recursive")
    if not _git(repo, "checkout", "--quiet", "--detach", msgs_commit):
        ui.abort("Could not check out px4_msgs")


def _rebuild(cfg):
    code = run_colcon(cfg, ["colcon", "build", "--symlink-install", "--packages-above", "px4_msgs"])
    if code != 0:
        raise typer.Exit(code)
    ui.ok("px4_msgs and its dependents rebuilt. PX4 SITL rebuilds on the next vespo simulate")


def in_sync(cfg):
    """Whether PX4-Autopilot and px4_msgs are at the pinned commits (no fetch)."""
    px4, repo = cfg.get("px4.path"), cfg.ws / "src" / "px4_msgs"
    msgs_version = _pinned_msgs(cfg)
    return (system.git_head(px4) == cfg.get("px4.commit")
            and bool(msgs_version) and system.git_head(repo) == system.git_resolve(repo, msgs_version))


def sync_machine(cfg, dry_run=False, rebuild=True):
    """
    Puts PX4-Autopilot and px4_msgs at the pinned commits. Fetches only when
    a commit is not there yet. Returns True if anything changed.
    """
    px4 = require_px4(cfg)
    repo = _msgs_repo(cfg)
    ref = cfg.get("px4.ref", "?")
    pinned = cfg.get("px4.commit")
    msgs_version = _pinned_msgs(cfg)
    if not pinned or not msgs_version:
        ui.abort("No PX4 pin in vespo.toml / vespo.repos. Set one with: vespo px4 update")

    ui.header(f"Syncing PX4 {ref}")
    commit = system.git_resolve(px4, pinned)
    if not commit:
        ui.info("Fetching PX4-Autopilot")
        if not _git(px4, "fetch", "--tags", "--quiet", "origin"):
            ui.abort("git fetch of PX4-Autopilot failed. Check the internet connection")
        commit = system.git_resolve(px4, pinned)
        if not commit:
            ui.abort(f"PX4-Autopilot has no commit {pinned[:10]} ({ref}). Is {px4} a clone of PX4/PX4-Autopilot?")
    msgs_commit = system.git_resolve(repo, msgs_version)
    if not msgs_commit:
        ui.info("Fetching px4_msgs")
        if not _git(repo, "fetch", "--quiet", "origin"):
            ui.abort("git fetch of px4_msgs failed. Check the internet connection")
        msgs_commit = system.git_resolve(repo, msgs_version)
        if not msgs_commit:
            ui.abort(f"px4_msgs has no commit {msgs_version[:10]}")

    px4_head, msgs_head = system.git_head(px4), system.git_head(repo)
    if px4_head == commit and msgs_head == msgs_commit:
        ui.ok(f"PX4-Autopilot and px4_msgs already at the team version {ref}")
        return False

    ui.info(f"PX4-Autopilot  {_describe(px4, px4_head)} -> {ref} ({commit[:10]})")
    ui.info(f"px4_msgs       {msgs_head[:10]} -> {msgs_commit[:10]}")
    if dry_run:
        ui.ok("Dry run, nothing changed")
        return False

    _checkout(px4, commit, repo, msgs_commit)
    ui.ok(f"PX4-Autopilot and px4_msgs at the team version {ref}")
    if rebuild:
        _rebuild(cfg)
    if px4_head != commit:
        ui.warn(f"Flash PX4 {ref} on the flight controllers too: vespo flash")
    return True


@app.command()
def sync(
    dry_run: Annotated[bool, typer.Option("--dry-run", help="Only show what would change.")] = False,
):
    """Put this machine's PX4 and px4_msgs at the team's pinned version and rebuild."""
    sync_machine(load_config(), dry_run)


def _choose_ref(cfg, px4, ref, main, release):
    if sum([ref is not None, main, release]) > 1:
        ui.abort("Use only one of REF, --main, --release")
    if main:
        return "origin/main"
    stable = latest_stable(px4)
    if release:
        if not stable:
            ui.abort("No stable vX.Y.Z tag found in PX4-Autopilot")
        return stable
    if ref:
        return ref
    # Enter keeps the team version: changing it must be a deliberate choice
    pinned_ref = cfg.get("px4.ref", "?")
    options = [(PINNED, f"Keep the team version {pinned_ref}, only sync this machine (same as vespo px4 sync)")]
    if stable and stable != pinned_ref:
        options.append((stable, "Latest stable release, changes the version for the whole team"))
    options.append(("origin/main", "Development branch, newest and least tested, changes it for the whole team"))
    return settings.choose("PX4 version", options, PINNED, None, False, "REF")


def _pin(cfg, ref, px4_commit, msgs_commit, msgs_branch):
    toml = cfg.project_path.read_text()
    toml = re.sub(r'(?m)^ref = ".*"$', f'ref = "{ref}"', toml, count=1)
    toml = re.sub(r'(?m)^commit = "[0-9a-f]+"$', f'commit = "{px4_commit}"', toml, count=1)
    cfg.project_path.write_text(toml)

    repos = cfg.repos_path.read_text()
    repos = re.sub(r"(px4_msgs:\n(?:\s+.*\n)*?\s+version: )\S+.*",
                   rf"\g<1>{msgs_commit}  # {msgs_branch}, same messages as PX4 {ref}", repos, count=1)
    cfg.repos_path.write_text(repos)


@app.command()
def update(
    ref: Annotated[Optional[str], typer.Argument(help="Tag, branch or commit, e.g. v1.17.0.", autocompletion=completion.px4_refs)] = None,
    main: Annotated[bool, typer.Option("--main", help="Newest development version.")] = False,
    release: Annotated[bool, typer.Option("--release", help="Newest stable vX.Y.Z tag.")] = False,
    dry_run: Annotated[bool, typer.Option("--dry-run", help="Only show what would change.")] = False,
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Change the team version without asking.")] = False,
):
    """Change the team's PX4 version: move PX4 and px4_msgs together, pin both and rebuild."""
    cfg = load_config()
    px4 = require_px4(cfg)
    repo = _msgs_repo(cfg)

    ui.header("Updating PX4")
    ui.info("Fetching PX4-Autopilot and px4_msgs")
    if not _git(px4, "fetch", "--tags", "--quiet", "origin") or not _git(repo, "fetch", "--quiet", "origin"):
        ui.abort("git fetch failed. Check the internet connection")

    ref = _choose_ref(cfg, px4, ref, main, release)
    if ref == PINNED:
        sync_machine(cfg, dry_run)
        return
    commit = system.git_resolve(px4, ref)
    if not commit:
        ui.abort(f"'{ref}' is not a tag, branch or commit of PX4-Autopilot")
    if commit == cfg.get("px4.commit"):
        # Same PX4 as the pin: keep the pinned px4_msgs too, don't re-pin
        sync_machine(cfg, dry_run)
        return
    label = ref.removeprefix("origin/")

    msgs_commit, branch = msgs.find_matching_commit(px4, commit, repo, msgs.candidate_branches(ref))
    if not msgs_commit:
        ui.abort(f"No px4_msgs commit has the same messages as PX4 {label} yet. "
                 "px4_msgs is generated with some delay; try again later or pick another version")

    ui.info(f"PX4-Autopilot  {_describe(px4, system.git_head(px4))} -> {label} ({commit[:10]})")
    ui.info(f"px4_msgs       {system.git_head(repo)[:10]} -> {msgs_commit[:10]} ({branch})")
    if dry_run:
        ui.ok("Dry run, nothing changed")
        return
    ui.warn(f"This changes the PX4 version of the whole team from {cfg.get('px4.ref', '?')} to {label}: "
            "vespo.toml and vespo.repos change, and every teammate has to run vespo px4 sync after pulling")
    if not settings.confirm("Change the team version?", yes or None, False, False):
        ui.info("Nothing changed. To only fix this machine: vespo px4 sync")
        return

    _checkout(px4, commit, repo, msgs_commit)
    _pin(cfg, label, commit, msgs_commit, branch)
    ui.ok(f"Pinned PX4 {label} in vespo.toml and px4_msgs in vespo.repos (commit them)")
    _rebuild(cfg)
    ui.warn(f"Flash PX4 {label} on the flight controllers too: vespo flash")
