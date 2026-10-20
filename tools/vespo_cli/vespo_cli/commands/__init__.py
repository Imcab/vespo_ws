"""One module per subcommand, registered in main.py. Shared helpers live here."""

from pathlib import Path

from .. import config, system, ui


def load_config():
    """The merged config, or a clean error instead of a traceback."""
    try:
        return config.load()
    except config.ConfigError as e:
        ui.abort(str(e))


def require_px4(cfg):
    """PX4-Autopilot path of this machine, or abort with the fix."""
    path = cfg.get("px4.path")
    if not path:
        ui.abort("PX4-Autopilot path is not set. Run: vespo config edit px4_path")
    if not system.is_px4_tree(path):
        ui.abort(f"{path} is not a PX4-Autopilot checkout. Run: vespo config edit px4_path")
    return Path(path)


def require_role(cfg):
    if not cfg.role:
        ui.abort("This machine has no role yet. Run: vespo init")
    return cfg.role
