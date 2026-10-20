"""
Matching px4_msgs to a PX4-Autopilot commit.

px4_msgs is generated from PX4's msg/ and srv/ folders (PX4 keeps versioned
messages in msg/versioned/, px4_msgs flattens them into msg/). Other
subfolders, such as msg/px4_msgs_old/ or the translation node tests, are not
part of px4_msgs and are left out. Two commits
carry the same messages when every .msg and .srv file name maps to the same
git blob hash, so the comparison is exact and needs no checkout.
"""

from pathlib import PurePosixPath

from .procs import capture

DIRS = ("msg", "srv")
FOLDERS = {"msg", "msg/versioned", "srv"}
SUFFIXES = (".msg", ".srv")


def message_blobs(repo, commit):
    """{file name: blob hash} of every message and service at `commit`, or None."""
    code, out = capture(["git", "-C", str(repo), "ls-tree", "-r", commit, "--", *DIRS], timeout=30)
    if code != 0:
        return None
    blobs = {}
    for line in out.splitlines():
        meta, _, path = line.partition("\t")
        parts = meta.split()
        if len(parts) == 3 and parts[1] == "blob":
            path = PurePosixPath(path)
            if str(path.parent) in FOLDERS and path.name.endswith(SUFFIXES):
                blobs[path.name] = parts[2]
    return blobs


def diff(px4_blobs, msgs_blobs):
    """(missing in px4_msgs, extra in px4_msgs, changed) file names."""
    missing = sorted(set(px4_blobs) - set(msgs_blobs))
    extra = sorted(set(msgs_blobs) - set(px4_blobs))
    changed = sorted(n for n in set(px4_blobs) & set(msgs_blobs) if px4_blobs[n] != msgs_blobs[n])
    return missing, extra, changed


def candidate_branches(px4_ref):
    """
    px4_msgs branches worth searching for a PX4 ref, most likely first:
    vX.Y.Z tags and release/X.Y branches map to release/X.Y, the rest to main.
    """
    ref = px4_ref.removeprefix("origin/")
    version = None
    if ref.startswith("v") and ref[1:2].isdigit():
        version = ".".join(ref[1:].split("-")[0].split(".")[:2])
    elif ref.startswith("release/"):
        version = ref.removeprefix("release/")
    branches = [f"release/{version}"] if version else []
    return branches + ["main"]


def find_matching_commit(px4_repo, px4_commit, msgs_repo, branches, depth=400):
    """
    Newest px4_msgs commit on any of `branches` (remote-tracking, so fetch
    first) whose messages equal PX4's at `px4_commit`. Returns (commit,
    branch) or (None, None).
    """
    target = message_blobs(px4_repo, px4_commit)
    if not target:
        return None, None
    for branch in branches:
        code, out = capture(
            ["git", "-C", str(msgs_repo), "rev-list", f"--max-count={depth}", f"origin/{branch}"], timeout=30,
        )
        if code != 0:
            continue
        for commit in out.splitlines():
            if message_blobs(msgs_repo, commit) == target:
                return commit, branch
    return None, None
