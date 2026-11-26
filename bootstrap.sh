#!/usr/bin/env bash
#
# The only step on a fresh Ubuntu 24.04 machine: installs ROS 2, the build
# tools and the vespo CLI, then hands over to `vespo init`, which installs
# PX4 and Gazebo if this machine simulates. Safe to run again.
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
UBUNTU_CODENAME_REQUIRED=noble   # the only Ubuntu with ROS 2 Jazzy packages
ROS_APT_SOURCE=https://github.com/ros-infrastructure/ros-apt-source/releases
APT_PKGS=(git curl wget lsb-release python3-pip ros-dev-tools "ros-$DISTRO-ros-base")

say() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
die() { printf '\033[1;31mxx\033[0m %s\n' "$*" >&2; exit 1; }

ASSUME_YES=0
for arg in "$@"; do
  [[ "$arg" == "--yes" || "$arg" == "-y" ]] && ASSUME_YES=1
done

confirm() {
  local answer=y
  (( ASSUME_YES )) || read -rp "$1 [Y/n] " answer
  [[ "${answer:-y}" == [Yy]* ]]
}

apt_install() {
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"
}

# Linux Mint and other derivatives say which Ubuntu they are built on here
codename() {
  ( . /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}" )
}

add_ros_repository() {
  say "Adding the ROS 2 apt repository"
  apt_install ca-certificates curl software-properties-common
  sudo add-apt-repository -y universe

  local url version deb
  url="$(curl -fsSLI -o /dev/null -w '%{url_effective}' "$ROS_APT_SOURCE/latest")"
  version="${url##*/}"
  [[ -n "$version" && "$version" != latest ]] || die "Could not find the latest ros2-apt-source release"
  deb="$(mktemp --suffix=.deb)"
  curl -fsSL -o "$deb" "$ROS_APT_SOURCE/download/$version/ros2-apt-source_$version.${UBUNTU_CODENAME_REQUIRED}_all.deb"
  sudo dpkg -i "$deb"
  rm -f "$deb"
}

say "Workspace: $WS"

found="$(codename)"
[[ "$found" == "$UBUNTU_CODENAME_REQUIRED" ]] \
  || die "vespo needs Ubuntu 24.04 (ROS 2 $DISTRO has no packages for '$found'). Install Ubuntu 24.04 and run this again."

missing=()
for pkg in "${APT_PKGS[@]}"; do
  dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if (( ${#missing[@]} )); then
  say "Missing system packages: ${missing[*]}"
  confirm "Install them with sudo apt now?" || die "Install them and run this script again."
  if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    # Older setups used this file, often with an expired key that fails apt-get
    # update. The ros2-apt-source package replaces it and apt refuses to have both
    sudo rm -f /etc/apt/sources.list.d/ros2.list
    sudo apt-get update
    add_ros_repository
  fi
  sudo apt-get update
  apt_install "${missing[@]}"
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  say "Initializing rosdep"
  sudo rosdep init
fi
say "Updating the rosdep database"
rosdep update --rosdistro "$DISTRO" >/dev/null || say "rosdep update failed, vespo init will warn about missing dependencies"

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
