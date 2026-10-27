#!/usr/bin/env bash
#
# First step on a fresh machine: checks the base system, installs the vespo
# CLI and hands over to `vespo init`. Safe to run again.
#
#   ./bootstrap.sh
#   ./bootstrap.sh --role companion --serial /dev/ttyTHS1 --yes
#
# Every argument is passed through to `vespo init`.

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISTRO="$(sed -n 's/^ros_distro *= *"\([^"]*\)".*/\1/p' "$WS/vespo.toml" | head -n1)"
DISTRO="${DISTRO:-jazzy}"
VENV="$WS/.venv"
APT_PKGS=(git python3-pip python3-colcon-common-extensions python3-vcstool python3-rosdep)

say() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
die() { printf '\033[1;31mxx\033[0m %s\n' "$*" >&2; exit 1; }

ASSUME_YES=0
for arg in "$@"; do
  [[ "$arg" == "--yes" || "$arg" == "-y" ]] && ASSUME_YES=1
done

say "Workspace: $WS"

command -v python3 >/dev/null || die "python3 not found"
python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))' \
  || die "Python 3.10+ is required, found $(python3 --version)"

[[ -f "/opt/ros/$DISTRO/setup.bash" ]] \
  || die "ROS 2 $DISTRO is not installed: https://docs.ros.org/en/$DISTRO/Installation.html"

missing=()
for pkg in "${APT_PKGS[@]}"; do
  dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if (( ${#missing[@]} )); then
  say "Missing system packages: ${missing[*]}"
  answer=y
  (( ASSUME_YES )) || read -rp "Install them with sudo apt now? [Y/n] " answer
  case "${answer:-y}" in
    [Yy]*) sudo apt-get update && sudo apt-get install -y "${missing[@]}" ;;
    *) die "Install them and run this script again." ;;
  esac
fi

# Ubuntu 24.04 forbids pip installs into the system Python (PEP 668), so the
# CLI gets its own environment. It is created without pip (that would need the
# python3-venv package) and filled by the system pip through --python. System
# site packages stay visible so apt's rich, click and yaml are reused.
if [[ ! -x "$VENV/bin/python" ]]; then
  say "Creating the CLI environment in .venv/"
  python3 -m venv --without-pip --system-site-packages "$VENV"
fi

say "Installing the vespo CLI"
python3 -m pip --python "$VENV/bin/python" install --quiet --disable-pip-version-check -e "$WS/tools/vespo_cli"

mkdir -p "$HOME/.local/bin"
ln -sf "$VENV/bin/vespo" "$HOME/.local/bin/vespo"

case ":$PATH:" in
  *":$HOME/.local/bin:"*) ;;
  *) say "~/.local/bin is not on PATH yet: vespo init adds it to ~/.bashrc, so it works in new terminals" ;;
esac

exec "$VENV/bin/vespo" init "$@"
