# vespo CLI

Command line tool for the VespoUAV ROS 2 + PX4 workspace. It sets up a
machine, builds what that machine needs and tells you what is broken.

## Fresh machine

From the workspace root:

```bash
./bootstrap.sh
```

It checks Python and ROS, installs missing system tools (`colcon`, `vcs`,
`rosdep`), installs this CLI into `.venv/` (linked as `~/.local/bin/vespo`)
and runs `vespo init`.

`vespo init` also installs Tab completion for bash
(`~/.local/share/bash-completion/completions/vespo`): commands, options and
their values, such as packages, models, worlds, settings, boards, serial
ports and PX4 versions. Values are read from the workspace on each Tab, so
new packages or worlds show up without reinstalling it.

Unattended, e.g. a Jetson over SSH:

```bash
./bootstrap.sh --role companion --serial /dev/ttyTHS1 --yes
```

## Commands

| Command | What it does |
|---|---|
| `vespo init` | Asks the role of the machine and what that role needs, writes `vespo.local.toml`, clones the pinned repos, installs dependencies with rosdep, builds and adds the workspace to `~/.bashrc`. Re-run it any time: previous answers are the defaults. |
| `vespo doctor [--live]` | Checks ROS, the workspace, the pinned repos, PX4, px4_msgs, Gazebo and the link to the flight controller, with a fix for each problem. `--live` also looks for `/fmu` topics. Exits 1 if anything failed. |
| `vespo build [PKG...] [--role R]` | `colcon build --symlink-install`, skipping the packages the role does not need. |
| `vespo new PKG [--cpp \| --python] [-t offboard\|listener] [-n NODE]` | Creates `src/PKG`, a ROS 2 package for PX4 with a first node and `launch/PKG.launch.py` (ready for `vespo up --launch PKG PKG.launch.py`), then builds it (`--no-build` to skip). `offboard` takes off in OFFBOARD and hovers at the `altitude` parameter (it arms the vehicle); `listener` only reads status, position and battery. On an existing package it adds a node instead (C++: the `NODES` list in CMakeLists.txt; Python: `console_scripts` in setup.py). PX4 topic names, including their `_vN` version suffix, are taken from the px4_msgs of the workspace. Templates are in `vespo_cli/templates/`. |
| `vespo clean PKG... [-r]` / `vespo clean --all` | Deletes `build/` and `install/` of those packages (or `build/`, `install/`, `log/` entirely); `-r` rebuilds. `vespo build` suggests it when a failure comes from an old non-symlink build. |
| `vespo agent [--serial DEV] [--udp] [-b BAUD] [-p PORT]` | Runs the Micro XRCE-DDS Agent: UDP for `sim`, the configured serial port for `companion`. The flags override the role, e.g. a Pixhawk on the bench from the sim PC. |
| `vespo simulate [--minimum \| --default \| --custom MODEL \| --show]` | Runs PX4 SITL in Gazebo Harmonic (`make px4_sitl gz_<model>`). `--minimum` flies the x500 in PX4's default world, `--default` (or nothing) uses the machine's `sim_*` settings, `--custom` any model with a PX4 SITL airframe, `--show` lists models and worlds. `--world` takes a PX4 world or one installed by a workspace package (`share/<pkg>/worlds/<world>.sdf`), which vespo starts in Gazebo before PX4 attaches to it. `--pose X,Y[,Z[,YAW]]` sets the spawn point (default: `<world>.spawn.yaml` or `spawn.yaml` next to a workspace world); `--follow METRES` / `--no-follow` set the camera; `--headless` / `--gui` the Gazebo window. |
| `vespo up [--sim \| --real]` | Starts everything needed to fly. `--sim`: agent over UDP + PX4 SITL in Gazebo + QGroundControl. `--real`: agent over serial/USB to the flight controller (`--serial DEV`, default the configured port or the first USB one) + QGroundControl. Without either the role decides: `sim` -> `--sim`, `companion` -> `--real` without QGroundControl. `--no-qgc` skips QGroundControl; `--headless`, `--world`, `--custom` are passed to `vespo simulate`; `--launch PKG FILE` also runs `ros2 launch PKG FILE`. In kitty each process gets its own pane in a new `vespo` tab; elsewhere (e.g. SSH) they run in this terminal with each line prefixed by the process name, and Ctrl+C stops them all. |
| `vespo down` | Stops everything `vespo up` started (Ctrl+C first, then harder) and any leftover PX4, Gazebo or agent process, from any terminal. |
| `vespo config` | Shows every machine setting and where its value comes from. |
| `vespo config set NAME VALUE` | Changes one setting, validated, without re-running init. |
| `vespo config edit [NAME]` | Asks one setting again from a menu. Changing the role asks for whatever the new role needs; changing `domain_id` updates `~/.bashrc`. |
| `vespo px4` | Shows the pinned PX4 version and whether PX4-Autopilot and px4_msgs match it. |
| `vespo px4 sync [--dry-run]` | Puts this machine's PX4-Autopilot (with its submodules) and px4_msgs at the versions pinned in `vespo.toml` / `vespo.repos` and rebuilds px4_msgs and its dependents. Never changes the pin. This is the fix whenever doctor says PX4 is not at the pinned version, e.g. after cloning PX4 at another version or pulling a new pin. `vespo init` offers it too. |
| `vespo px4 update [REF \| --main \| --release] [--dry-run] [--yes]` | Changes the PX4 version of the whole team: moves PX4-Autopilot to a version and px4_msgs to the newest commit with exactly the same messages (compared by git blob hash), pins both in `vespo.toml` / `vespo.repos` (commit them) and rebuilds. Asks before changing the pin. Without arguments it offers a menu whose default keeps the team version (same as `sync`). |
| `vespo qgc [-f]` | Launches QGroundControl in the background (log in `~/.cache/vespo/qgc.log`); `-f` keeps it attached to the terminal. Warns about missing packages and ModemManager. |
| `vespo qgc status` | Where the AppImage is, its version, the latest release on GitHub, and whether the system packages, ModemManager and the `dialout` group are right. |
| `vespo qgc install [--path P] [--force]` | Downloads the latest QGroundControl AppImage for this machine (x86_64 or aarch64), or updates the installed one, and saves it as `qgc_path`. |
| `vespo flash [--no-upload] [--target T]` | `make <target> upload` in PX4-Autopilot for the configured flight controller. `--no-upload` only builds. |

## Roles

Defined in `vespo.toml` under `[roles.*]`:

| Role | Machine | Agent link | Simulation | QGroundControl |
|---|---|---|---|---|
| `sim` | Development PC | UDP 8888 | PX4 SITL + Gazebo Harmonic | Yes |
| `companion` | Jetson / Raspberry Pi on the drone | Serial to the Pixhawk | No | No |
| `ground` | Monitoring laptop | None | No | Yes |

## Panes in kitty

`vespo up` opens its panes through kitty remote control, which is off by
default. Add to `~/.config/kitty/kitty.conf` and restart kitty:

```
allow_remote_control yes
```

Without it `vespo up` says so and runs everything in the current terminal.

## Configuration

| File | Committed | Holds |
|---|---|---|
| `vespo.toml` | Yes | Project defaults: ROS distro, PX4 version, ports, roles |
| `vespo.local.toml` | No | This machine: role, PX4 path, serial port, domain ID |
| `vespo.repos` | Yes | Third-party repos pinned to exact commits |

Local values override project values key by key.

Settings (`vespo config` names): `role`, `domain_id`, `flight_controller`,
`px4_path`, `serial`, `baud`, `qgc_path`, and for simulation `sim_model`, `sim_world`,
`sim_pose`, `sim_follow`, `sim_headless` (what `vespo simulate --default`
runs; team defaults in `vespo.toml` under `[simulation]`). They are defined
once in `vespo_cli/settings.py` and shared by `init` and `config`.

Flight controllers are listed in `vespo.toml` under `[flight_controllers]`,
each with the PX4 make target `vespo flash` builds. Adding a board is one line.

## Adding a command

1. Create `vespo_cli/commands/<name>.py` with a function named after the command.
2. Register it in `vespo_cli/main.py` with `app.command()(<module>.<function>)`.
3. Read settings with `load_config()` from `commands/__init__.py` and print through `ui`, like the others.
