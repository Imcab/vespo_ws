# VespoUAV workspace

ROS 2 Jazzy + PX4 v1.17 workspace of the VespoUAV autonomous drone team.

| | |
|---|---|
| ROS 2 | Jazzy, Ubuntu 24.04 |
| PX4 | v1.17.0 (pinned in `vespo.toml`) |
| Simulator | Gazebo Harmonic |
| Bridge | Micro XRCE-DDS Agent v2.4.3 |

## Getting started

Before, on Ubuntu 24.04:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
  in `/opt/ros/jazzy`
- On a machine that simulates (role `sim`), PX4-Autopilot at the pinned
  version, which also installs Gazebo Harmonic. `vespo init` asks for its path:

  ```bash
  git clone --recursive -b v1.17.0 https://github.com/PX4/PX4-Autopilot.git
  bash PX4-Autopilot/Tools/setup/ubuntu.sh
  ```

Then:

```bash
git clone <this repo> vespo_ws && cd vespo_ws
./bootstrap.sh
```

`bootstrap.sh` installs the `vespo` command and runs `vespo init`, which adds
it to `~/.bashrc`. The terminal that ran it does not see the command yet:
open a new one (or run `source ~/.bashrc`), then:

```bash
vespo doctor        # is everything in place?
vespo agent         # terminal 1: bridge PX4 <-> ROS 2
vespo simulate      # terminal 2: PX4 SITL in Gazebo
```

## Layout

```
vespo.toml      project settings, PX4 version, roles (committed)
vespo.repos     third-party repos pinned to exact commits (committed)
src/            ROS 2 packages; px4_msgs and the agent come from vespo.repos
tools/          vespo CLI (not a colcon package)
```

PX4-Autopilot lives outside the workspace (e.g. `../PX4-Autopilot`); each
machine sets its path with `vespo init`.

See [tools/vespo_cli/README.md](tools/vespo_cli/README.md) for every command.
