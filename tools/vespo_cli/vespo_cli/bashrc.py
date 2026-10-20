"""
The block vespo keeps in ~/.bashrc: workspace overlay, ROS_DOMAIN_ID and
VESPO_WS. It sits between two marker lines and is rewritten as a whole, so
running init again never duplicates lines or touches anything else.

Tab completion is not part of the block: its script goes where
bash-completion looks for commands, and is loaded the first time Tab is
pressed after `vespo`.
"""

import os
import re
import shlex
import shutil
import sys
from pathlib import Path

from .procs import capture

BEGIN = "# >>> vespo >>>"
END = "# <<< vespo <<<"
BASHRC = Path.home() / ".bashrc"
COMPLETION = (Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local" / "share")
              / "bash-completion" / "completions" / "vespo")
BLOCK = re.compile(rf"\n?{re.escape(BEGIN)}.*?{re.escape(END)}\n?", re.S)


def block(cfg):
    ws = shlex.quote(str(cfg.ws))
    return "\n".join([
        BEGIN,
        "# Written by `vespo init` / `vespo config`. Edits here are overwritten.",
        f"export VESPO_WS={ws}",
        f"export ROS_DOMAIN_ID={int(cfg.get('ros.domain_id', 0))}",
        f"[ -f {ws}/install/setup.bash ] && source {ws}/install/setup.bash",
        END,
    ])


def installed():
    try:
        return BEGIN in BASHRC.read_text()
    except OSError:
        return False


def write(cfg):
    """Adds or replaces the vespo block. Returns True if the file changed."""
    try:
        text = BASHRC.read_text()
    except FileNotFoundError:
        text = ""
    new_block = block(cfg)
    if BEGIN in text:
        updated = BLOCK.sub("\n" + new_block + "\n", text, count=1)
    else:
        updated = text.rstrip("\n") + "\n\n" + new_block + "\n"
    if updated == text:
        return False
    BASHRC.write_text(updated)
    return True


def install_completion():
    """Writes the bash completion script. Returns True if it changed, None on failure."""
    vespo = shutil.which("vespo") or str(Path(sys.argv[0]).resolve())
    code, script = capture([vespo], env={"_VESPO_COMPLETE": "source_bash"})
    if code != 0 or "complete " not in script:
        return None
    if COMPLETION.is_file() and COMPLETION.read_text() == script + "\n":
        return False
    COMPLETION.parent.mkdir(parents=True, exist_ok=True)
    COMPLETION.write_text(script + "\n")
    return True
