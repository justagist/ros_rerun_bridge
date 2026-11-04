# Installation

The bridge needs a sourced ROS 2 environment (tested with Jazzy and Lyrical) and its own Python environment.

## Why its own environment

Rerun needs NumPy 2, while Jazzy ships NumPy 1. The bridge therefore runs in its own virtual environment and takes ROS's Python packages (`rclpy`, message types) from the sourced ROS environment through `PYTHONPATH`. The bridge's environment:

* must not be the system Python: on Jazzy, upgrading NumPy there breaks other ROS nodes (e.g. OpenCV, `cv_bridge`);
* must not include system site-packages: on Jazzy, the distro's NumPy 1 builds of packages such as pandas then break Rerun;
* must use the Python your ROS distro is built for (3.12 on Jazzy, 3.14 on Lyrical), as `rclpy` is compiled for it.

Lyrical ships NumPy 2, but the same environment still keeps Rerun's pinned dependencies apart from the system's.

[uv](https://docs.astral.sh/uv/) handles all of this. The repo tells uv to use a Python installed on the system, never one it downloads.

## ROS packages for some modules

A few modules need ROS packages that a minimal ROS install (`ros-base`) leaves out. Install them only for the modules you use:

| Module | Packages (`$ROS_DISTRO` is set once ROS is sourced) |
| --- | --- |
| `pointcloud2` | `ros-${ROS_DISTRO}-sensor-msgs-py` |
| `laserscan` | `ros-${ROS_DISTRO}-laser-geometry`, `ros-${ROS_DISTRO}-sensor-msgs-py` |
| `particle_cloud` | `ros-${ROS_DISTRO}-nav2-msgs` |
| `detections_2d`, `detections_3d` | `ros-${ROS_DISTRO}-vision-msgs` |

## From a clone, with `uv run`

```bash
source /opt/ros/jazzy/setup.bash  # or lyrical
# From the repo root: creates .venv on first run, then runs the bridge
uv run ros-rerun-bridge --config examples/robot_arm/config.yaml
```

## As a command available everywhere

Install it as a uv tool. It gets its own environment, and the command goes in `~/.local/bin` (run `uv tool update-shell` if that is not on your `PATH`). `uv tool` does not read the repo's settings, so ask for a system Python explicitly:

```bash
# From a clone, editable: code changes apply without reinstalling (add --reinstall after dependency changes)
uv tool install --python-preference only-system -e .
# Or from GitHub
uv tool install --python-preference only-system git+https://github.com/justagist/ros_rerun_bridge

source /opt/ros/jazzy/setup.bash  # or lyrical
ros-rerun-bridge --config <config.yaml>

# To remove it
uv tool uninstall ros-rerun-bridge
```

To run once without installing, `uvx` works the same way:

```bash
uvx --python-preference only-system --from git+https://github.com/justagist/ros_rerun_bridge ros-rerun-bridge --config <config.yaml>
```

## The Rerun viewer

The bridge's environment includes the Rerun viewer, at the bridge's Rerun version, and the bridge starts it from there. To run it yourself, e.g. to open a saved `.rrd` file or [view a bridge on another machine](REMOTE_VIEWING.md):

```bash
# From a clone
uv run rerun <file.rrd or url>
# From a uv tool install (see `uv tool dir` for the location)
~/.local/share/uv/tools/ros-rerun-bridge/bin/rerun <file.rrd or url>
```

Or add `--with-executables-from rerun-sdk` to `uv tool install` to put `rerun` in `~/.local/bin` too. Another `rerun` earlier on your `PATH` would take precedence, and may not match the bridge's Rerun version.

## With pip

In a plain virtual environment created from the distro's Python (on Ubuntu, this needs the `python3-venv` package):

```bash
/usr/bin/python3 -m venv .venv && .venv/bin/pip install -e .
.venv/bin/ros-rerun-bridge --config <config.yaml>
```
