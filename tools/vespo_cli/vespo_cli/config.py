"""
Loads the project config (vespo.toml) merged with the machine config
(vespo.local.toml), and writes the machine config back.

vespo.toml is committed and identical on every machine. vespo.local.toml is
written by `vespo init`, ignored by git, and overrides the project values.
"""

import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


PROJECT_FILE = "vespo.toml"
LOCAL_FILE = "vespo.local.toml"
REPOS_FILE = "vespo.repos"

LOCAL_HEADER = """\
# Machine-specific settings written by `vespo init`. Not committed.
# Run `vespo init` again to change them, or edit by hand.
"""


class ConfigError(Exception):
    """The workspace or one of its config files cannot be used."""


@dataclass
class Config:
    ws: Path
    data: dict   # project merged with local
    local: dict  # local only, as read from disk

    @property
    def project_path(self):
        return self.ws / PROJECT_FILE

    @property
    def local_path(self):
        return self.ws / LOCAL_FILE

    @property
    def repos_path(self):
        return self.ws / REPOS_FILE

    @property
    def distro(self):
        return self.get("workspace.ros_distro", "jazzy")

    @property
    def role(self):
        return self.get("machine.role")

    def roles(self):
        return self.get("roles", {})

    def role_spec(self, role=None):
        """Returns the [roles.<name>] table of the given role, or of this machine."""
        return self.roles().get(role or self.role or "", {})

    def get(self, key, default=None):
        """Dotted lookup, e.g. get('agent.udp_port')."""
        node = self.data
        for part in key.split("."):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node


def find_workspace():
    """
    Locates the workspace root, the folder holding vespo.toml.

    Order: $VESPO_WS, then the current directory and its parents, then the
    location of this file (the CLI is installed editable from <ws>/tools/).
    A $VESPO_WS without vespo.toml is ignored rather than fatal, because old
    shells may still export it for a previous workspace; `vespo doctor`
    reports it.
    """
    candidates = []
    env = os.environ.get("VESPO_WS")
    if env:
        candidates.append(Path(env).expanduser())
    cwd = Path.cwd()
    candidates += [cwd, *cwd.parents]
    candidates.append(Path(__file__).resolve().parents[3])

    for path in candidates:
        if (path / PROJECT_FILE).is_file():
            return path.resolve()
    raise ConfigError(
        f"No {PROJECT_FILE} found. Run vespo from inside the workspace "
        "or set VESPO_WS to its root."
    )


def load(ws=None):
    ws = ws or find_workspace()
    project = _read(ws / PROJECT_FILE)
    local_path = ws / LOCAL_FILE
    local = _read(local_path) if local_path.is_file() else {}
    return Config(ws=ws, data=_merge(project, local), local=local)


def write_local(ws, data):
    """
    Writes vespo.local.toml. Supports one level of [sections] holding
    strings, numbers, booleans and flat lists, which is all init produces.
    """
    lines = [LOCAL_HEADER]
    for section, values in data.items():
        lines.append(f"[{section}]")
        for key, value in values.items():
            lines.append(f"{key} = {_toml_value(value)}")
        lines.append("")
    (ws / LOCAL_FILE).write_text("\n".join(lines))


def set_key(data, key, value):
    """Sets a dotted 'section.name' key in a two-level dict, in place."""
    section, _, name = key.partition(".")
    data.setdefault(section, {})[name] = value


def save_local_key(cfg, key, value):
    """Changes one key of vespo.local.toml, keeping every other value."""
    local = {section: dict(values) for section, values in cfg.local.items()}
    set_key(local, key, value)
    write_local(cfg.ws, local)


def _read(path):
    try:
        with open(path, "rb") as f:
            return tomllib.load(f)
    except OSError as e:
        raise ConfigError(f"Cannot read {path}: {e.strerror}") from e
    except tomllib.TOMLDecodeError as e:
        raise ConfigError(f"Invalid TOML in {path}: {e}") from e


def _merge(base, override):
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _toml_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    if isinstance(value, str):
        # JSON string escapes are valid TOML basic-string escapes
        return json.dumps(value)
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_toml_value(v) for v in value) + "]"
    raise TypeError(f"Cannot write {type(value).__name__} to TOML")
