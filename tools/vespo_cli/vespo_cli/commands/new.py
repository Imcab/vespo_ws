"""
`vespo new`: a ROS 2 package for PX4 in src/, in C++ or Python, with a first
node from a template and a launch file ready for `vespo up --launch`.
Run on an existing package, it adds another node to it.

Templates live in vespo_cli/templates: cpp/ and python/ are the package
skeletons, nodes/ the node templates. Placeholders:

  @PKG@ @NODE@ @CLASS@ @TITLE@ @DESCRIPTION@ @MAINTAINER@ @EMAIL@
  @out:VehicleStatus@         -> /fmu/out/vehicle_status_v1
  @in:TrajectorySetpoint@     -> /fmu/in/trajectory_setpoint

PX4 appends _v<N> to a topic when its message has MESSAGE_VERSION N > 0
(uxrce_dds_client/utilities.hpp), so topics are resolved from the px4_msgs
in this workspace instead of hard-coded.

A template can need packages beyond rclcpp and px4_msgs (the `mode` template
needs px4_ros2_cpp). They are added to package.xml and to the DEPENDENCIES
list of CMakeLists.txt, and a library missing from the workspace is offered
for cloning, pinned in vespo.repos to the branch matching the pinned PX4.
"""

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Optional
from xml.sax.saxutils import escape as xml_escape

import typer

from .. import completion, procs, settings, ui
from . import load_config
from .build import run_colcon

TEMPLATES = Path(__file__).resolve().parent.parent / "templates"
NAME = re.compile(r"^[a-z][a-z0-9_]*$")
TOPIC = re.compile(r"@(in|out):([A-Za-z0-9]+)@")
VERSION = re.compile(r"^\s*uint32\s+MESSAGE_VERSION\s*=\s*(\d+)", re.M)

LANGUAGES = {"cpp": ("C++", "cpp"), "python": ("Python", "py")}


@dataclass
class NodeTemplate:
    label: str
    node: str                          # default node name
    languages: tuple = ("cpp", "python")
    deps: tuple = ()                   # packages beyond rclcpp and px4_msgs
    arms: bool = True                  # arms the vehicle: warn


NODE_TEMPLATES = {
    "offboard": NodeTemplate("OFFBOARD takeoff and hover", "offboard_control"),
    "listener": NodeTemplate("Only reads status, position and battery", "telemetry_listener", arms=False),
    "takeoff_land": NodeTemplate("Takeoff, wait and land with PX4's own modes (no offboard)", "takeoff_land"),
    "waypoints": NodeTemplate("OFFBOARD mission through a list of waypoints, then land", "waypoint_mission"),
    "orbit": NodeTemplate("OFFBOARD circle with position + velocity setpoints", "orbit"),
    "mode": NodeTemplate("Own PX4 flight mode + executor, listed in QGroundControl (px4-ros2-interface-lib)",
                         "square_mode", languages=("cpp",), deps=("px4_ros2_cpp",)),
}

# Packages vespo can clone when a template needs them: package -> (repo folder, url)
LIBRARIES = {
    "px4_ros2_cpp": ("px4-ros2-interface-lib", "https://github.com/Auterion/px4-ros2-interface-lib.git"),
}
MODE_NAME_MAX = 24   # px4_ros2: mode names must be shorter than 25 characters


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def _snake(camel):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", camel).lower()


def _camel(snake):
    return "".join(part.capitalize() for part in snake.split("_"))


def _title(snake):
    return " ".join(part.capitalize() for part in snake.split("_"))


def px4_topic(cfg, direction, message):
    """ROS topic PX4 uses for a message, e.g. ('out', 'VehicleStatus') -> /fmu/out/vehicle_status_v1."""
    msg = cfg.ws / "src" / "px4_msgs" / "msg" / f"{message}.msg"
    if not msg.is_file():
        ui.abort(f"px4_msgs has no {message}.msg (src/px4_msgs missing? Run: vespo init)")
    match = VERSION.search(msg.read_text(errors="ignore"))
    version = int(match.group(1)) if match else 0
    return f"/fmu/{direction}/{_snake(message)}" + (f"_v{version}" if version else "")


def _render(cfg, text, values):
    for key, value in values.items():
        text = text.replace(f"@{key}@", value)
    return TOPIC.sub(lambda m: px4_topic(cfg, m.group(1), m.group(2)), text)


def _escaped(values, relative):
    """Free text (description, maintainer) escaped for the file it goes into."""
    free = ("DESCRIPTION", "MAINTAINER", "EMAIL")
    if relative.endswith(".xml"):
        return {**values, **{k: xml_escape(values[k]) for k in free}}
    if relative.endswith(".py"):
        return {**values, **{k: values[k].replace("\\", "\\\\").replace("'", "\\'") for k in free}}
    return values


def _write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def _node_file(cfg, lang, template, values):
    ext = LANGUAGES[lang][1]
    source = TEMPLATES / "nodes" / (f"{template}.{ext}" + (".tmpl" if ext == "py" else ""))
    return _render(cfg, source.read_text(), values)


def _maintainer(cfg):
    _, name = procs.capture(["git", "-C", str(cfg.ws), "config", "user.name"])
    _, email = procs.capture(["git", "-C", str(cfg.ws), "config", "user.email"])
    return name or "TODO", email or "todo@todo.todo"


# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

def _available(cfg, pkg):
    return bool(_find_package(cfg, pkg) or (cfg.ws / "install" / pkg).is_dir()
                or Path(f"/opt/ros/{cfg.distro}/share/{pkg}").is_dir())


def _library_branch(cfg):
    """Branch of px4-ros2-interface-lib for the pinned PX4: v1.17.x -> release/1.17, else main."""
    match = re.match(r"^v(\d+)\.(\d+)\.", str(cfg.get("px4.ref", "")))
    return f"release/{match.group(1)}.{match.group(2)}" if match else "main"


def _ensure_library(cfg, pkg, yes):
    """
    Clones the repo providing `pkg` into src/ and pins it in vespo.repos, if
    the workspace does not have it yet. Returns True when it was added.
    """
    if _available(cfg, pkg):
        return False
    repo, url = LIBRARIES[pkg]
    branch = _library_branch(cfg)
    code, out = procs.capture(["git", "ls-remote", url, f"refs/heads/{branch}"], timeout=30)
    if code != 0:
        ui.abort(f"Could not reach {url} (offline?)")
    if not out:
        ui.abort(f"{repo} has no {branch} branch for PX4 {cfg.get('px4.ref')}. "
                 f"Add it to vespo.repos by hand at a version matching px4_msgs")
    commit = out.split()[0]

    ui.info(f"{pkg} comes from {repo}, branch {branch} (matches PX4 {cfg.get('px4.ref')})")
    if not settings.confirm(f"Add {repo} to vespo.repos and clone it into src/?", None, yes, True):
        ui.abort(f"The template needs {pkg}")
    target = cfg.ws / "src" / repo
    if target.exists():
        ui.abort(f"{target} exists but does not provide {pkg}")
    ui.info(f"Cloning {repo}")
    if procs.capture(["git", "clone", "--quiet", "--branch", branch, url, str(target)], timeout=600)[0] != 0 \
            or procs.capture(["git", "-C", str(target), "checkout", "--quiet", "--detach", commit])[0] != 0:
        ui.abort(f"git clone of {url} failed")

    repos = cfg.repos_path.read_text().rstrip("\n")
    cfg.repos_path.write_text(repos + f"""

  # C++ API for custom PX4 flight modes and mode executors (vespo new -t mode).
  # {branch} matches PX4 {cfg.get('px4.ref')}: move it together with PX4 and px4_msgs.
  # Its example_* packages are skipped by `vespo build`.
  {repo}:
    type: git
    url: {url}
    version: {commit}  # {branch}
""")
    ui.ok(f"Cloned src/{repo} at {commit[:10]} and pinned it in vespo.repos (commit it)")
    return True


def _add_dependencies(folder, lang, deps):
    """Adds packages to package.xml and, for C++, to the DEPENDENCIES list of CMakeLists.txt."""
    if not deps:
        return
    xml_path = folder / "package.xml"
    xml = xml_path.read_text()
    cmake_path = folder / "CMakeLists.txt"
    if lang == "cpp":
        cmake = cmake_path.read_text()
        block = re.search(r"^set\(DEPENDENCIES\b.*?^\)", cmake, re.S | re.M)
        if not block:
            ui.abort(f"{cmake_path} has no set(DEPENDENCIES ...) list. Add {', '.join(deps)} by hand "
                     "(find_package, ament_target_dependencies and <depend> in package.xml)")
    for dep in deps:
        if not re.search(rf"<(\w+_)?depend>\s*{dep}\s*</", xml):
            # After the last plain <depend>, else after any *_depend
            lines = (list(re.finditer(r"^[ \t]*<depend>.*$", xml, re.M))
                     or list(re.finditer(r"^[ \t]*<\w+_depend>.*$", xml, re.M)))
            last = lines[-1]
            xml = xml[:last.end()] + f"\n  <depend>{dep}</depend>" + xml[last.end():]
        if lang == "cpp" and not re.search(rf"^\s*{dep}\s*$", block.group(0), re.M):
            cmake = cmake[:block.end() - 1] + f"  {dep}\n" + cmake[block.end() - 1:]
            block = re.search(r"^set\(DEPENDENCIES\b.*?^\)", cmake, re.S | re.M)
    xml_path.write_text(xml)
    if lang == "cpp":
        cmake_path.write_text(cmake)


# ---------------------------------------------------------------------------
# Existing packages
# ---------------------------------------------------------------------------

def _find_package(cfg, name):
    """Folder of the package called `name` in src/, or None."""
    for xml in (cfg.ws / "src").glob("**/package.xml"):
        if re.search(rf"<name>\s*{re.escape(name)}\s*</name>", xml.read_text(errors="ignore")):
            return xml.parent
    return None


def _language_of(folder):
    if (folder / "CMakeLists.txt").is_file() and re.search(r"^set\(NODES\b", (folder / "CMakeLists.txt").read_text(), re.M):
        return "cpp"
    if (folder / "setup.py").is_file() and "console_scripts" in (folder / "setup.py").read_text():
        return "python"
    return None


def _add_node(cfg, folder, pkg, lang, template, node):
    values = {"PKG": pkg, "NODE": node, "CLASS": _camel(node), "TITLE": _title(node)}
    _add_dependencies(folder, lang, NODE_TEMPLATES[template].deps)
    if lang == "cpp":
        target = folder / "src" / f"{node}.cpp"
        cmake = folder / "CMakeLists.txt"
        text = cmake.read_text()
        block = re.search(r"^set\(NODES\b.*?^\)", text, re.S | re.M)
        if re.search(rf"^\s*{node}\b", block.group(0), re.M):
            ui.abort(f"{node} is already in the NODES list of {cmake}. Pass another name with --node")
        if target.exists():
            ui.abort(f"{target} already exists")
        _write(target, _node_file(cfg, lang, template, values))
        cmake.write_text(text[:block.end() - 1] + f"  {node}\n" + text[block.end() - 1:])
        ui.ok(f"Added src/{node}.cpp and listed it in CMakeLists.txt")
    else:
        target = folder / pkg / f"{node}.py"
        setup = folder / "setup.py"
        text = setup.read_text()
        if re.search(rf"['\"]{node}\s*=", text):
            ui.abort(f"{node} is already a console script in {setup}. Pass another name with --node")
        if target.exists():
            ui.abort(f"{target} already exists")
        _write(target, _node_file(cfg, lang, template, values))
        # After the last entry of the console_scripts list, with its indent
        entry = re.search(r"['\"]console_scripts['\"]\s*:\s*\[[ \t]*\n", text)
        close = re.compile(r"^([ \t]*)\]", re.M).search(text, entry.end())
        indent = close.group(1) + " " * 4
        setup.write_text(text[:close.start()] + f"{indent}'{node} = {pkg}.{node}:main',\n" + text[close.start():])
        ui.ok(f"Added {pkg}/{node}.py and its entry point in setup.py")


# ---------------------------------------------------------------------------
# Command
# ---------------------------------------------------------------------------

def new(
    name: Annotated[str, typer.Argument(help="Package name (snake_case). An existing one gets a new node.", autocompletion=completion.packages)],
    cpp: Annotated[bool, typer.Option("--cpp", help="C++ package (ament_cmake).")] = False,
    python: Annotated[bool, typer.Option("--python", help="Python package (ament_python).")] = False,
    template: Annotated[Optional[str], typer.Option("--template", "-t", help=", ".join(NODE_TEMPLATES) + ".", autocompletion=lambda incomplete: [(t, spec.label) for t, spec in NODE_TEMPLATES.items() if t.startswith(incomplete)])] = None,
    node: Annotated[Optional[str], typer.Option("--node", "-n", help="Node name (default from the template).")] = None,
    description: Annotated[Optional[str], typer.Option("--description", "-d", help="One line for package.xml.")] = None,
    build: Annotated[bool, typer.Option("--build/--no-build", help="Build it right away.")] = True,
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Defaults instead of questions.")] = False,
):
    """Create a ROS 2 package for PX4 (C++ or Python), or add a node to one."""
    cfg = load_config()
    if cpp and python:
        ui.abort("Use either --cpp or --python")
    if not NAME.match(name):
        ui.abort(f"'{name}' is not a valid package name: lowercase letters, digits and _, starting with a letter")
    if node is not None and not NAME.match(node):
        ui.abort(f"'{node}' is not a valid node name: lowercase letters, digits and _, starting with a letter")

    existing = _find_package(cfg, name)
    if existing:
        lang = _language_of(existing)
        if not lang:
            ui.abort(f"{existing} exists but has no NODES list (CMakeLists.txt) or console_scripts (setup.py) "
                     "to add a node to")
        if (cpp and lang != "cpp") or (python and lang != "python"):
            ui.abort(f"{name} is a {LANGUAGES[lang][0]} package")
        ui.header(f"Adding a node to {name} ({LANGUAGES[lang][0]})")
    else:
        lang = "cpp" if cpp else "python" if python else settings.choose(
            "Language", [(key, label) for key, (label, _) in LANGUAGES.items()], "cpp", None, yes, "--cpp/--python")
        ui.header(f"New {LANGUAGES[lang][0]} package {name}")

    choices = [(key, spec.label) for key, spec in NODE_TEMPLATES.items() if lang in spec.languages]
    if template in NODE_TEMPLATES and lang not in NODE_TEMPLATES[template].languages:
        ui.abort(f"The {template} template is C++ only: the Python API of px4-ros2-interface-lib "
                 "only exists for PX4 main, not for the PX4 release this workspace is pinned to")
    template = settings.choose("First node" if not existing else "Node", choices, "offboard", template, yes, "--template")
    spec = NODE_TEMPLATES[template]
    node = node or spec.node
    if "mode" in template and len(_title(node)) > MODE_NAME_MAX:
        ui.abort(f"The mode name '{_title(node)}' (from the node name) must be at most {MODE_NAME_MAX} characters")
    added_library = any([_ensure_library(cfg, dep, yes) for dep in spec.deps if dep in LIBRARIES])

    if existing:
        _add_node(cfg, existing, name, lang, template, node)
        folder = existing
    else:
        folder = cfg.ws / "src" / name
        if folder.exists():
            ui.abort(f"{folder} already exists")
        maintainer, email = _maintainer(cfg)
        values = {
            "PKG": name, "NODE": node, "CLASS": _camel(node), "TITLE": _title(node),
            "MAINTAINER": maintainer, "EMAIL": email,
            "DESCRIPTION": description or f"ROS 2 nodes for PX4: {name}",
        }
        skeleton = TEMPLATES / lang
        for source in sorted(p for p in skeleton.rglob("*") if p.is_file()):
            relative = _render(cfg, str(source.relative_to(skeleton)), values).removesuffix(".tmpl")
            _write(folder / relative, _render(cfg, source.read_text(), _escaped(values, relative)))
        node_path = folder / ("src" if lang == "cpp" else name) / f"{node}.{LANGUAGES[lang][1]}"
        _write(node_path, _node_file(cfg, lang, template, values))
        _add_dependencies(folder, lang, spec.deps)
        ui.ok(f"Created src/{name} with the {template} node {node}")

    if spec.arms:
        ui.warn(f"{node} ARMS the vehicle: try it in simulation first, and without propellers on the real one")
    if template == "mode":
        ui.info(f"Start it from QGroundControl: flight mode '{_title(node)}' (or: commander mode ext1 in the PX4 shell)")
    if build:
        # A freshly cloned library has to be built first
        select = "--packages-up-to" if added_library else "--packages-select"
        code = run_colcon(cfg, ["colcon", "build", "--symlink-install", select, name])
        if code != 0:
            raise typer.Exit(code)
        ui.ok("Built")
    else:
        ui.info(f"Build it with: vespo build {name}")
    ui.info(f"Run it:  ros2 run {name} {node}   (with vespo up running)")
    if not existing:
        ui.info(f"Or all together:  vespo up --launch {name} {name}.launch.py")
