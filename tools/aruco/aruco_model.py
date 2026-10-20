"""

Module that renders the Gazebo model files (model.config and model.sdf)
for a given ArUco marker, out of the templates in aruco_template/.

Every path and name comes from aruco_paths, so this module never decides
where a file goes or how it is called.

Author: Imad Jared Cabrera Trejo
Contact: cabreratrejoimadjared@gmail.com
Team: VespoUAV

"""

from aruco_ui import fail
from aruco_paths import (
    CONFIG_TEMPLATE,
    SDF_TEMPLATE,
    get_marker_name,
    get_config_path,
    get_sdf_path,
    get_texture_uri,
)


def generate_aruco_config(marker_id):
    """
    Renders the model.config of a marker and writes it inside its model folder.

    Parameters:
    - marker_id: the arucotag id to generate the .config file for

    The folder is expected to exist already (save_aruco_markers creates it).
    """
    try:
        # Get the content of the xml template file for it to be modified with the actual marker ID
        content = CONFIG_TEMPLATE.read_text(encoding="utf-8")
    except FileNotFoundError:
        fail(f"Template not found: {CONFIG_TEMPLATE}")
        return

    # Replace the placeholders in the template with this marker's values
    rendered = content.format(
        model_name=get_marker_name(marker_id),
        tag_id=marker_id,
    )

    # Now we go to our output file and write out our output
    output_file = get_config_path(marker_id)
    output_file.write_text(rendered, encoding="utf-8")


def generate_aruco_sdf(marker_id, size_meters):
    """
    Renders the model.sdf of a marker and writes it inside its model folder.

    Parameters:
    - marker_id: the arucotag id to generate the .sdf file for
    - size_meters: the physical side length of the tag, in meters
                   (the TMR rulebook specifies 0.20 m)

    The folder is expected to exist already (save_aruco_markers creates it).
    """
    try:
        # Get the content of the xml template file for it to be modified with the actual marker ID
        content = SDF_TEMPLATE.read_text(encoding="utf-8")
    except FileNotFoundError:
        fail(f"Template not found: {SDF_TEMPLATE}")
        return

    # Replace the placeholders in the template with this marker's values.
    # The texture URI is built by aruco_paths so it can never drift apart
    # from the folder and file names actually used on disk.
    rendered = content.format(
        model_name=get_marker_name(marker_id),
        size_meters=size_meters,
        texture_uri=get_texture_uri(marker_id),
    )

    # Now we go to our output file and write out our output
    output_file = get_sdf_path(marker_id)
    output_file.write_text(rendered, encoding="utf-8")