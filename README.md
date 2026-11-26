# VespoUAV workspace

ROS 2 Jazzy + PX4 v1.17 workspace of the VespoUAV autonomous drone team.

| | |
|---|---|
| ROS 2 | Jazzy, Ubuntu 24.04 |
| PX4 | v1.17.0 (pinned in `vespo.toml`) |
| Simulator | Gazebo Harmonic |
| Bridge | Micro XRCE-DDS Agent v2.4.3 |

## Getting started

You only need Ubuntu 24.04 and git (`sudo apt install git`):

```bash
git clone https://github.com/Imcab/vespo_ws.git && cd vespo_ws
./bootstrap.sh
```

`bootstrap.sh` installs everything else: ROS 2 Jazzy, the build tools and the
`vespo` command. Then `vespo init` asks the role of the machine and sets it up:
on a `sim` machine that means PX4 at the team version, Gazebo Harmonic and
QGroundControl. A first install downloads several GB and takes a while; if it
stops, run `./bootstrap.sh` again and it continues.

The terminal that ran it does not see the command yet: open a new one (or run
`source ~/.bashrc`), then:

```bash
vespo doctor        # is everything in place?
vespo agent         # terminal 1: bridge PX4 <-> ROS 2
vespo simulate      # terminal 2: PX4 SITL in Gazebo
```

## Updates

`vespo update` gets the team's latest version and rebuilds only what changed.
You don't have to remember it: vespo checks in the background and tells you
when there is something new, and updates that only touch the CLI apply by
themselves. See [tools/vespo_cli/README.md](tools/vespo_cli/README.md#updates).

## Layout

```
vespo.toml      project settings, PX4 version, roles (committed)
vespo.repos     third-party repos pinned to exact commits (committed)
src/            ROS 2 packages; px4_msgs and the agent come from vespo.repos
tools/          vespo CLI (not a colcon package)
```

PX4-Autopilot lives outside the workspace, next to it (`../PX4-Autopilot`)
unless `vespo init` is told otherwise.

## PX4 version

Everyone uses the PX4 pinned in `vespo.toml`. If `vespo doctor` says PX4 or
px4_msgs is not at the pinned version (e.g. PX4 was cloned from `main`, which
is 1.18), run `vespo px4 sync`: it checks out the pinned version on this
machine and rebuilds, without changing the pin. `vespo px4 update` is only for
moving the whole team to another PX4 version.

See [tools/vespo_cli/README.md](tools/vespo_cli/README.md) for every command.
