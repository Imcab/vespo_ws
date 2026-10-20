"""

Module with the small terminal helpers used to make the CLI feel alive:
colours, status lines and an in place progress bar.

Everything here is plain stdlib, no external dependency, so the tooling keeps
working on a fresh machine with nothing but ROS and OpenCV installed.

Author: Imad Jared Cabrera Trejo
Contact: cabreratrejoimadjared@gmail.com
Team: VespoUAV

"""

import shutil
import sys
import time

# ANSI escape codes. \033[0m resets everything back to the terminal default,
# so every helper below ends with it.
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
CYAN = "\033[36m"

# Characters used to draw the bar. The filled one is a full block and the
# empty one is a light shade, so the bar reads well even without colour.
BAR_FILLED = "█"
BAR_EMPTY = "░"

# How wide the bar itself is, in characters
BAR_WIDTH = 30


def header(text):
    """
    Prints a bold blue banner, used for the title and for section changes.
    """
    print(f"{BOLD}{BLUE}----- {text} -----{RESET}")


def ok(text):
    """
    Prints a success line with a green check.
    """
    print(f"  {GREEN}[SUCCESS]{RESET} {text}")


def fail(text):
    """
    Prints a failure line with a red cross.
    """
    print(f"  {RED}[FAIL]{RESET} {text}")


def info(text):
    """
    Prints a dimmed informational line, for things that are not results.
    """
    print(f"  {DIM}{text}{RESET}")


def warn(text):
    """
    Prints a yellow warning line.
    """
    print(f"  {YELLOW}!{RESET} {text}")


def _render_bar(label, done, total):
    """
    Builds the bar string for a given progress state.

    Kept private because callers should use progress() instead of drawing
    the bar themselves.
    """
    # Guard against a division by zero on an empty job
    fraction = done / total if total else 1.0

    filled = int(BAR_WIDTH * fraction)
    bar = BAR_FILLED * filled + BAR_EMPTY * (BAR_WIDTH - filled)
    percent = int(fraction * 100)

    return f"  {label} {CYAN}{bar}{RESET} {percent:3d}%  ({done}/{total})"


def progress(items, label):
    """
    Wraps an iterable and draws a progress bar that updates in place.

    Use it in a for loop exactly like the original iterable:

        for marker_id in progress(marker_ids, "Writing markers"):
            ...

    Parameters:
    - items: any sequence (it must have a length, so pass a list, not a generator)
    - label: short text shown to the left of the bar

    The bar is redrawn on the same line using a carriage return, and a final
    newline is printed once the loop is done.
    """
    total = len(items)

    # Draw the empty bar first so the user sees something immediately,
    # even if the first item takes a while
    sys.stdout.write("\r" + _render_bar(label, 0, total))
    sys.stdout.flush()

    for index, item in enumerate(items, start=1):
        # Hand the item to the caller's loop body
        yield item

        # The body has finished with this item, so redraw with the new count.
        # \r moves the cursor back to the start of the line, which is what
        # makes the bar update in place instead of printing a new line.
        sys.stdout.write("\r" + _render_bar(label, index, total))
        sys.stdout.flush()

        # A tiny pause so the bar is actually visible on fast jobs.
        # Without it, generating 5 markers finishes before anything is drawn.
        time.sleep(0.02)

    # Close the line the bar was living on
    sys.stdout.write("\n")
    sys.stdout.flush()


def clear_line():
    """
    Erases the current terminal line, useful before printing an error in the
    middle of a progress bar.
    """
    width = shutil.get_terminal_size().columns
    sys.stdout.write("\r" + " " * width + "\r")
    sys.stdout.flush()