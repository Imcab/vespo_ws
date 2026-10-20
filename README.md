# VespoUAV workspace

ROS 2 Jazzy + PX4 v1.17 workspace of the VespoUAV autonomous drone team.

| | |
|---|---|
| ROS 2 | Jazzy, Ubuntu 24.04 |
| PX4 | v1.17.0 (pinned in `vespo.toml`) |
| Simulator | Gazebo Harmonic |
| Bridge | Micro XRCE-DDS Agent v2.4.3 |

## Getting started

```bash
git clone <this repo> vespo_ws && cd vespo_ws
./bootstrap.sh
```

Then:

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
