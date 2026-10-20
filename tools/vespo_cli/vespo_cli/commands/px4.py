"""
`vespo px4`: which PX4 this workspace is pinned to, and moving that pin.

PX4-Autopilot and px4_msgs always move together: `update` checks PX4 out at
a version, finds the px4_msgs commit with exactly the same messages
(msgs.find_matching_commit), pins both in vespo.toml / vespo.repos and
rebuilds px4_msgs and everything that depends on it.
"""

import re
from typing import Annotated, Optional

import typer

from .. import completion, msgs, procs, settings, system, ui
from . import load_config, require_px4
from .build import run_colcon

app = typer.Typer(
    help="Show or change the PX4 version of the workspace.",
    context_settings={"help_option_names": ["-h", "--help"]},
)

STABLE_TAG = re.compile(r"^v(\d+)\.(\d+)\.(\d+)$")


def _msgs_repo(cfg):
    path = cfg.ws / "src" / "px4_msgs"
    if not (path / ".git").exists():
        ui.abort("src/px4_msgs is missing. Run: vespo init (or: vcs import src < vespo.repos)")
    return path


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
        ui.warn(f"PX4-Autopilot at {_describe(px4, head)}, not the pinned commit. Fix: vespo px4 update {pinned[:10]}")

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
        ui.info("Fix: vespo px4 update")


def _choose_ref(px4, ref, main, release):
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
    options = [(stable, "Latest stable release (recommended)")] if stable else []
    options.append(("origin/main", "Development branch, newest and least tested"))
    return settings.choose("PX4 version", options, stable, None, False, "REF")


def _pin(cfg, ref, px4_commit, msgs_commit, msgs_branch):
    toml = cfg.project_path.read_text()
    toml = re.sub(r'(?m)^ref = ".*"$', f'ref = "{ref}"', toml, count=1)
    toml = re.sub(r'(?m)^commit = "[0-9a-f]+"$', f'commit = "{px4_commit}"', toml, count=1)
    cfg.project_path.write_text(toml)

    repos = cfg.repos_path.read_text()
    repos = re.sub(r"(px4_msgs:\n(?:\s+.*\n)*?\s+version: )\S+.*",
                   rf"\g<1>{msgs_commit}  # {msgs_branch}, same messages as PX4 {ref}", repos, count=1)
    cfg.repos_path.write_text(repos)


def _git(repo, *args, timeout=600):
    code, _ = procs.capture(["git", "-C", str(repo), *args], timeout=timeout)
    return code == 0


@app.command()
def update(
    ref: Annotated[Optional[str], typer.Argument(help="Tag, branch or commit, e.g. v1.17.0.", autocompletion=completion.px4_refs)] = None,
    main: Annotated[bool, typer.Option("--main", help="Newest development version.")] = False,
    release: Annotated[bool, typer.Option("--release", help="Newest stable vX.Y.Z tag.")] = False,
    dry_run: Annotated[bool, typer.Option("--dry-run", help="Only show what would change.")] = False,
):
    """Move PX4 and px4_msgs to a version together and rebuild."""
    cfg = load_config()
    px4 = require_px4(cfg)
    repo = _msgs_repo(cfg)

    ui.header("Updating PX4")
    ui.info("Fetching PX4-Autopilot and px4_msgs")
    if not _git(px4, "fetch", "--tags", "--quiet", "origin") or not _git(repo, "fetch", "--quiet", "origin"):
        ui.abort("git fetch failed. Check the internet connection")

    ref = _choose_ref(px4, ref, main, release)
    commit = system.git_resolve(px4, ref)
    if not commit:
        ui.abort(f"'{ref}' is not a tag, branch or commit of PX4-Autopilot")
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

    code, dirty = procs.capture(["git", "-C", str(px4), "status", "--porcelain", "--untracked-files=no"])
    if dirty:
        ui.abort("PX4-Autopilot has uncommitted changes. Commit or stash them first")

    if not _git(px4, "checkout", "--quiet", "--detach", commit):
        ui.abort("Could not check out PX4-Autopilot")
    ui.info("Updating PX4 submodules (this can take a few minutes)")
    if not (_git(px4, "submodule", "sync", "--recursive")
            and _git(px4, "submodule", "update", "--init", "--recursive", "--jobs", "8", timeout=1800)):
        ui.abort("Updating the PX4 submodules failed. Run in PX4-Autopilot: git submodule update --init --recursive")
    if not _git(repo, "checkout", "--quiet", "--detach", msgs_commit):
        ui.abort("Could not check out px4_msgs")

    _pin(cfg, label, commit, msgs_commit, branch)
    ui.ok(f"Pinned PX4 {label} in vespo.toml and px4_msgs in vespo.repos (commit them)")

    code = run_colcon(cfg, ["colcon", "build", "--symlink-install", "--packages-above", "px4_msgs"])
    if code != 0:
        raise typer.Exit(code)
    ui.ok("px4_msgs and its dependents rebuilt. PX4 SITL rebuilds on the next vespo simulate")
    ui.warn(f"Flash PX4 {label} on the flight controllers too: vespo flash")
