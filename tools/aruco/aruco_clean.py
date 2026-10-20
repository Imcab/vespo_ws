"""

Module that removes generated artifacts: the Gazebo model folders and,
optionally, the per-world tags.yaml files.

Everything it deletes lives under paths that come from aruco_paths, and it
only ever removes folders matching the marker naming convention, so it can
never wander outside the package.

Author: Imad Jared Cabrera Trejo
Contact: cabreratrejoimadjared@gmail.com
Team: VespoUAV

"""

import shutil

from aruco_paths import MODELS_DIR, WORLDS_DIR, MARKER_PREFIX

def find_marker_dirs():
    """
    Returns the list of generated marker model folders currently on disk.

    Only folders named after the marker convention are returned, so a stray
    file or an unrelated folder inside MODELS_DIR is never touched.
    """
    if not MODELS_DIR.is_dir():
        return []

    return sorted(
        entry for entry in MODELS_DIR.glob(f"{MARKER_PREFIX}_*") if entry.is_dir()
    )


def find_tags_configs():
    """
    Returns the list of tags.yaml files currently on disk, one per world.
    """
    if not WORLDS_DIR.is_dir():
        return []

    return sorted(WORLDS_DIR.glob("*/tags.yaml"))


def confirm(question):
    """
    Asks a yes/no question and returns True only on an explicit yes.

    Anything other than 'y' counts as no, so an accidental Enter never deletes.
    """
    answer = input(f"{question} (y/n): ").lower().strip()
    return answer == "y"


def clean_markers():
    """
    Deletes every generated marker model folder, after confirmation.

    The PNG, model.config and model.sdf inside each folder are all generated,
    so nothing handwritten is lost. tags.yaml is NOT touched.
    """
    marker_dirs = find_marker_dirs()

    if not marker_dirs:
        print("Nothing to clean: there are no generated markers.")
        return

    print(f"\nThis will delete {len(marker_dirs)} marker folder(s) from {MODELS_DIR}:")
    for marker_dir in marker_dirs:
        print(f"  - {marker_dir.name}")

    if not confirm("\nDelete them?"):
        print("Cancelled, nothing was deleted.")
        return

    for marker_dir in marker_dirs:
        shutil.rmtree(marker_dir)

    print(f"Deleted {len(marker_dirs)} marker folder(s).")


def clean_all():
    """
    Deletes every generated marker model folder AND every world's tags.yaml.

    This one does lose handwritten work: the poses you edited in tags.yaml
    are gone, so it asks separately for them.
    """
    clean_markers()

    tags_configs = find_tags_configs()

    if not tags_configs:
        print("No tags.yaml files found.")
        return

    print(f"\nThis will also delete {len(tags_configs)} tags.yaml file(s):")
    for config in tags_configs:
        print(f"  - {config}")
    print("\nWARNING: any pose you edited by hand in these files will be lost.")

    if not confirm("\nDelete them too?"):
        print("Cancelled, the tags.yaml files were kept.")
        return

    for config in tags_configs:
        config.unlink()

    print(f"Deleted {len(tags_configs)} tags.yaml file(s).")