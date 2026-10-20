"""

Single source of truth for every path and naming convention used by the
ArUco tooling. Every other script imports from here so that moving a folder
or renaming a file only has to be done in one place.

Author: Imad Jared Cabrera Trejo
Contact: cabreratrejoimadjared@gmail.com
Team: VespoUAV

"""

from pathlib import Path


# ---------------------------------------------------------------------------
# Base directories
# ---------------------------------------------------------------------------

# This module lives in <workspace>/tools/aruco/, so:
#   parent      -> tools/aruco   (where the templates live, next to the scripts)
#   parents[2]  -> the workspace root
TOOL_DIR = Path(__file__).parent
WORKSPACE_ROOT = Path(__file__).parents[2]

# Everything Gazebo consumes (models and worlds) lives in the description
# package, NOT next to these scripts. The tooling is a generator; the package
# is the deliverable.
DESCRIPTION_PKG = WORKSPACE_ROOT / "src" / "simulated_env"

# Where the .template files live. Kept next to the scripts that render them,
# and OUTSIDE of MODELS_DIR on purpose: Gazebo scans MODELS_DIR looking for
# models, and a template folder in there would be picked up as a broken model.
TEMPLATES_DIR = TOOL_DIR / "aruco_template"

# Templates themselves
CONFIG_TEMPLATE = TEMPLATES_DIR / "config.template"
SDF_TEMPLATE = TEMPLATES_DIR / "sdf.template"

# Where the generated Gazebo models are written. This is the directory that
# must be exported in GZ_SIM_RESOURCE_PATH, because Gazebo looks for models
# as DIRECT children of the paths in that variable.
MODELS_DIR = DESCRIPTION_PKG / "models" / "aruco_markers"

# Where world files and their configuration live
WORLDS_DIR = DESCRIPTION_PKG / "worlds"

# Physical side length of every ArUco tag, in meters.
# Fixed by the TMR rulebook: the marker next to the whiteboard is 20 cm x 20 cm.
# It is a constant and not a per-world setting on purpose: a marker has a single
# model.sdf on disk, so it cannot have two different sizes at the same time.
MARKER_SIZE_METERS = 0.20

def get_tags_config_path(world_name):
    """
    Returns the path of the YAML holding the tag poses for a given world,
    e.g. <vespo_description>/worlds/practice/tags.yaml
    """
    return get_world_dir(world_name) / "tags.yaml"


def get_world_dir(world_name):
    """
    Returns the directory of a world, e.g. <vespo_description>/worlds/practice/
    Each world keeps its own tags.yaml and its own .sdf file.
    """
    return WORLDS_DIR / world_name


def get_world_file(world_name):
    """
    Returns the path of a world's SDF file,
    e.g. <vespo_description>/worlds/practice/practice.sdf
    """
    return get_world_dir(world_name) / f"{world_name}.sdf"


# ---------------------------------------------------------------------------
# Per-marker naming conventions
# ---------------------------------------------------------------------------

# Prefix shared by the model folder, the model name and the texture file,
# so a single marker is always referred to the same way everywhere.
MARKER_PREFIX = "arucotag"


def get_marker_name(marker_id):
    """
    Returns the canonical name of a marker, e.g. 'arucotag_100'.

    This is the name of its folder, the <name> in its model.config and the
    <model name=...> in its model.sdf. It is also what model:// resolves against.
    """
    return f"{MARKER_PREFIX}_{marker_id}"


def get_marker_dir(marker_id):
    """
    Returns the directory of a marker's Gazebo model,
    e.g. <vespo_description>/models/aruco_markers/arucotag_100/
    """
    return MODELS_DIR / get_marker_name(marker_id)


def get_texture_filename(marker_id):
    """
    Returns just the file name of a marker's PNG, e.g. 'arucotag_100.png'.
    Used by the SDF template to build the albedo_map URI.
    """
    return f"{get_marker_name(marker_id)}.png"


def get_texture_path(marker_id):
    """
    Returns the full path where a marker's PNG is written, e.g.
    <vespo_description>/models/aruco_markers/arucotag_100/arucotag_100.png
    """
    return get_marker_dir(marker_id) / get_texture_filename(marker_id)


def get_config_path(marker_id):
    """
    Returns the full path of a marker's model.config file.
    """
    return get_marker_dir(marker_id) / "model.config"


def get_sdf_path(marker_id):
    """
    Returns the full path of a marker's model.sdf file.
    """
    return get_marker_dir(marker_id) / "model.sdf"


def get_texture_uri(marker_id):
    """
    Returns the model:// URI Gazebo uses to find a marker's texture,
    e.g. 'model://arucotag_100/arucotag_100.png'

    This is what goes inside <albedo_map> in the generated model.sdf.
    """
    return f"model://{get_marker_name(marker_id)}/{get_texture_filename(marker_id)}"
