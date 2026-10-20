"""

Module that handles everything related to ArUco marker images:
obtaining the dictionary, generating/saving the PNG files and
drawing a preview tag on the terminal.

Author: Imad Jared Cabrera Trejo
Contact: cabreratrejoimadjared@gmail.com
Team: VespoUAV

"""

import cv2 # OpenCV library for computer vision tasks
import cv2.aruco as aruco # Submodule of OpenCV for ArUco marker detection/generation

from aruco_paths import get_marker_dir, get_texture_path
from aruco_ui import progress, ok, fail, clear_line

# Single source of truth for the ArUco family used by the team.
# DICT_5X5_250 is a predefined dictionary of 250 markers with a 5x5 grid.
# 5x5 because of TMR's rules, and 250 because the rulebook uses marker ID 100,
# which does not exist in DICT_5X5_100 (that one only goes up to 99). See ->
# https://femexrobotica.org/tmr2026/index.php/categorias/categorias-fmr/drones-autonomos/
FAMILY_NAME = "DICT_5X5_250"
FAMILY_SIZE = 250 # number of unique markers in the dictionary

# Valid marker IDs go from 0 to FAMILY_SIZE - 1
MIN_MARKER_ID = 0
MAX_MARKER_ID = FAMILY_SIZE - 1


def get_family():
    """
    Returns a human readable label of the ArUco family in use.
    """
    return f"Family: {FAMILY_NAME}"


def get_family_range():
    """
    Returns the valid marker ID range as a string, ready to be shown in prompts.
    """
    return f"({MIN_MARKER_ID}-{MAX_MARKER_ID})"


def get_family_limits():
    """
    Returns the valid marker ID limits as a (minimum, maximum) tuple.
    """
    return (MIN_MARKER_ID, MAX_MARKER_ID)


def get_aruco_dictionary():
    """
    Returns the ArUco dictionary used by the team.
    """
    return aruco.getPredefinedDictionary(aruco.DICT_5X5_250)


def draw_aruco_on_terminal(aruco_dict, marker_id=0):
    """
    Draws an ArUco marker on the terminal as ASCII art, used as a welcome tag.

    Parameters:
    - aruco_dict: The ArUco dictionary to use for marker generation.
    - marker_id: The marker ID to draw (0 by default).
    """
    # A small marker is enough, we only want the bit pattern, not a real image
    img = aruco.generateImageMarker(aruco_dict, marker_id, 25)

    # Turn the 0/255 grayscale image into a 0/1 matrix
    img_matrix = img // 255

    # We print two block characters per pixel because terminal characters
    # are taller than they are wide, so a single one looks squashed
    for row in img_matrix:
        print("".join("██" if pixel else "  " for pixel in row))


def save_aruco_markers(markers_ids, aruco_dict, marker_size):
    """
    Function to generate and save ArUco markers based on the provided marker IDs.

    Each marker gets its own Gazebo model folder, and its PNG is written inside
    that folder. The exact locations come from aruco_paths, so this function
    does not decide any path or file name by itself.

    Parameters:
    - markers_ids: List of marker IDs to generate.
    - aruco_dict: The ArUco dictionary to use for marker generation.
    - marker_size: Size of the markers in pixels.
    """
    failed = []

    for marker_id in progress(markers_ids, "Rendering textures "):
        # Generate the marker image using the ArUco dictionary and the specified size in pixels
        generated_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

        # Each marker lives in its own model folder, create it if it does not exist.
        # This is also the folder where model.config and model.sdf will be written.
        get_marker_dir(marker_id).mkdir(parents=True, exist_ok=True)

        # The texture path (folder + file name) is a shared convention, so we ask
        # aruco_paths for it instead of building the name here
        filename = get_texture_path(marker_id)

        # We cast filename to str because cv2.imwrite() accepts only str as input, not a Path object managed by pathlib
        success = cv2.imwrite(str(filename), generated_img)
        if not success:
            failed.append(marker_id)

    # Report once at the end instead of one line per marker, so the progress
    # bar is not broken up by output while it is running
    if failed:
        fail(f"{len(failed)} texture(s) could not be written: {failed}")
    else:
        ok(f"{len(markers_ids)} texture(s) written")