#!/usr/bin/env python3

# The #!/usr/bin/env python3 is called a shebang line. 
# It tells the operating system to use the Python 3 interpreter to run this script. 
# This is important because it ensures that the script will be executed with the correct version of Python,
# regardless of the user's environment or default settings.

"""

Script to generate ArUco markers using OpenCV.
Run this script from: workspace_aruco/src/aruco_detector_pkg/scripts/aruco_generator.py

Author: Imad Jared Cabrera Trejo
Contact: cabreratrejoimadjared@gmail.com
Team: VespoUAV

"""

import sys

from aruco_img import (
    get_aruco_dictionary,
    draw_aruco_on_terminal,
    save_aruco_markers,
    get_family,
    get_family_limits,
)
from aruco_clean import clean_markers, clean_all
from aruco_model import generate_aruco_config, generate_aruco_sdf
from aruco_yaml import write_tags_config
from aruco_paths import MODELS_DIR, MARKER_SIZE_METERS, get_tags_config_path
from aruco_ui import header, ok, info, warn, progress


def ask_marker_count(max_id):
    """
    Question 1. Asks how many markers to generate and validates the answer.

    Returns the number of markers as an int.
    """
    while True:
        try:
            # Get user input for the number of markers to generate
            num_markers = int(input(f"How many markers do you want to generate? (1-{max_id + 1}): "))

            # Validate the input to ensure it's within the acceptable range
            if(num_markers >= 1 and num_markers <= max_id + 1):
                return num_markers
            else:
                print(f"Please enter a number between 1 and {max_id + 1}.")
        except ValueError:
            # Handle the case where the input is not a valid integer
            print("Invalid input. Please enter a valid integer.")
            continue


def ask_marker_size():
    """
    Question 2. Asks for the marker size in pixels and validates the answer.

    Returns the size as an int.
    """
    while True:
        try:
            print("The size of the markers is in pixels, for example 100 means 100x100 pixels")
            return int(input("What size do you want the markers to be? (in pixels): "))
        except ValueError:
            # Handle the case where the input is not a valid integer
            print("Invalid input. Please enter a valid integer.")
            continue


def ask_is_consecutive():
    """
    Question 3. Asks whether the marker IDs should be consecutive.

    Returns True for consecutive IDs, False if the user wants to pick them.
    """
    while True:
        # example consecutive markers: 0,1,2,3,4,5,6,7,8,9,10
        # the other option will trigger a submenu to ask for the specific marker IDs to generate
        # the method .lower is used to convert the input to lowercase so Y and y are treated the same
        # finally the method .strip is used to remove any blank spaces
        consecutive = input("Do you want the markers to be consecutive? (y/n): ").lower().strip()

        # In this case there is no exception handling because we manually handle the exception
        # by checking the input and ensure it never throws an exception.
        if(consecutive == "y"):
            return True
        elif(consecutive == "n"):
            return False
        else:
            print("Invalid input. Please enter 'y' or 'n'.")
            continue


def ask_specific_ids(num_markers, min_id, max_id):
    """
    Question 4. Asks for a comma separated list of specific marker IDs
    and validates range, duplicates and count.

    Returns the list of IDs.
    """
    while True:
        try:
            header(f"Enter the {num_markers} marker IDs you want")
            info(f"IDs must be between {min_id} and {max_id}, and duplicates are not allowed.")
            marker_ids_input = input("Please enter the specific marker IDs you want to generate (separated by commas, e.g., 1,2,3): ")

            # Split the input string by commas, strip whitespace, and convert to integers (if not empty)
            marker_ids = [int(id.strip()) for id in marker_ids_input.split(",") if id.strip()]

            # First validation, if the markers are within the family limits
            markers_inrange = all(id >= min_id and id <= max_id for id in marker_ids)

            # Second validation, duplicate IDs are not allowed
            has_duplicates = len(marker_ids) != len(set(marker_ids)) # set removes duplicates,
            # so if the lengths are different, there are duplicates

            # Third validation, if the number of markers is exactly the same as the number of markers requested by the user
            is_correct_number = len(marker_ids) == num_markers # here we dont use set because for total validation
            # the duplicates one will throw false so the validation will not be completed

            if (markers_inrange and not has_duplicates and is_correct_number):
                return marker_ids # all validations passed

            # If not valid, we print an error message and ask again
            if not markers_inrange:
                print(f"Error: All marker IDs must be between {min_id} and {max_id}.")
            if has_duplicates:
                print("Error: Duplicate marker IDs are not allowed.")
            if not is_correct_number:
                print(f"Error: You must enter exactly {num_markers} marker IDs.")
            continue

        except ValueError:
            # Handle the case where the input is not a valid integer
            print("Invalid input. Please enter valid integers separated by commas.")
            continue


def ask_world_name():
    """
    Question 0. Asks which world these markers belong to.

    Each world keeps its own tags.yaml, so the same marker can be placed
    differently in a practice world and in the competition one.

    Returns the world name as a string.
    """
    while True:
        world_name = input("Which world are these markers for? (e.g. practice, tmr_final): ").strip()

        # A world name becomes a folder name, so reject anything that would
        # make an awkward path
        if not world_name:
            print("Please enter a world name.")
            continue
        if "/" in world_name or "\\" in world_name or " " in world_name:
            print("Please use a name without spaces or slashes.")
            continue

        return world_name


def ask_main_action():
    """
    Main menu. Asks what the user wants to do before anything else happens.

    Returns the chosen option as a string: "1", "2", "3" or "4".
    """
    while True:
        print("\nWhat do you want to do?")
        print("  1 - Clean markers      (delete the generated model folders)")
        print("  2 - Clean everything   (also delete every world's tags.yaml)")
        print("  3 - Generate tags")
        print("  4 - Exit")

        action = input("Choose an option (1-4): ").strip()

        if action in ("1", "2", "3", "4"):
            return action

        print("Invalid input. Please enter 1, 2, 3 or 4.")


def generate_tags(aruco_dict, min_id, max_id):

    # Ask everything the generation needs
    world_name = ask_world_name()
    num_markers = ask_marker_count(max_id)
    marker_size = ask_marker_size()
    consecutive = ask_is_consecutive()

    # Both branches only DECIDE which IDs to generate. The generation itself
    # happens once, below, so we never duplicate it.
    if consecutive:
        marker_ids = list(range(num_markers))
    else:
        marker_ids = ask_specific_ids(num_markers, min_id, max_id)

    # Step 1. Write the PNG of every marker. This also creates each model folder.
    save_aruco_markers(marker_ids, aruco_dict, marker_size)

    # Step 2. Write the model.config and model.sdf of every marker inside the
    # folder just created. The physical size is a constant from the TMR rulebook,
    # so the SDF can be written right away.
    for marker_id in progress(marker_ids, "Building models   "):
        generate_aruco_config(marker_id)
        generate_aruco_sdf(marker_id, MARKER_SIZE_METERS)

    ok(f"{len(marker_ids)} Gazebo model(s) built")

    # Step 3. Register the markers in this world's tags.yaml. Poses already
    # edited by hand are preserved; only brand new markers get a placeholder.
    new_ids = write_tags_config(world_name, marker_ids)

    config_path = get_tags_config_path(world_name)
    if new_ids:
        ok(f"{len(new_ids)} marker(s) added to {config_path.name}: {new_ids}")
        warn("Their poses are all zeros. Edit them before generating the world.")
    else:
        info(f"No new markers, {config_path.name} left untouched.")

    print()
    header(f"Done. Models are in {MODELS_DIR}")


def main():

    # obtain the ArUco dictionary used by the team
    aruco_dict = get_aruco_dictionary()

    # Valid marker IDs for the family in use, so the prompts and the validations
    # stay in sync automatically if we ever switch dictionaries
    min_id, max_id = get_family_limits()

    # Start the menu
    print()
    header("VespoUAV Aruco Marker Generator")

    # Draw a sample marker in the console (for visual reference)
    draw_aruco_on_terminal(aruco_dict)

    info(get_family())

    # The menu keeps running so the user can clean and then generate
    # without having to launch the script again
    while True:
        action = ask_main_action()

        if action == "1":
            clean_markers()
        elif action == "2":
            clean_all()
        elif action == "3":
            generate_tags(aruco_dict, min_id, max_id)
        elif action == "4":
            print("Bye.")
            sys.exit(0) # Exit with a success status code (0)


if __name__ == "__main__":
    main() # Run the menu when the script is executed directly