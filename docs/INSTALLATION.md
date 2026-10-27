# Installation

The bridge needs a sourced ROS 2 environment (tested with Jazzy) and its own Python environment.

## Why its own environment

Rerun needs NumPy 2, while ROS distros ship NumPy 1. The bridge therefore runs in its own virtual environment and takes ROS's Python packages (`rclpy`, message types) from the sourced ROS environment through `PYTHONPATH`. The bridge's environment:

* must not be the system Python: upgrading NumPy there breaks other ROS nodes (e.g. OpenCV, `cv_bridge`);
* must not include system site-packages: the distro's NumPy 1 builds of packages such as pandas then break Rerun;
* must use the Python your ROS distro is built for (3.12 on Jazzy), as `rclpy` is compiled for it.

[uv](https://docs.astral.sh/uv/) handles all of this. The repo tells uv to use a Python installed on the system, never one it downloads.

## From a clone, with `uv run`

```bash
source /opt/ros/jazzy/setup.bash
# From the repo root: creates .venv on first run, then runs the bridge
uv run ros-rerun-bridge --config example_configs/robot_joint_states.config.yaml
```

## As a command available everywhere

Install it as a uv tool. It gets its own environment, and the command goes in `~/.local/bin` (run `uv tool update-shell` if that is not on your `PATH`). `uv tool` does not read the repo's settings, so ask for a system Python explicitly:

```bash
# From a clone, editable: code changes apply without reinstalling (add --reinstall after dependency changes)
uv tool install --python-preference only-system -e .
# Or from GitHub
uv tool install --python-preference only-system git+https://github.com/justagist/ros_rerun_bridge

source /opt/ros/jazzy/setup.bash
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
