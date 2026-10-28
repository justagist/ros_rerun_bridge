# ros-rerun-bridge

[![CI](https://github.com/justagist/ros_rerun_bridge/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/justagist/ros_rerun_bridge/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Rerun 0.40](https://img.shields.io/badge/Rerun-0.40-0B8CE9.svg)](https://rerun.io)
[![uv](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json)](https://github.com/astral-sh/uv)

Show live ROS 2 data in the [Rerun](https://rerun.io) viewer, as you would in rviz. List the topics you want to see in a YAML config file: the bridge subscribes to them and sends each message to Rerun, drawing spatial data in its TF frame.

> **Status:** I created this repository mainly to get familiar with Rerun and to use it in my own ROS projects. It is under active development, so features, interfaces and behaviour may change. Tested with Rerun 0.40.0 and ROS 2 Jazzy.

## Demos

**Nav2:** the [Nav2 getting started demo](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/) with [this config](examples/nav2_turtlebot3/config.yaml).

[![ros_rerun_tb3_sim.mp4](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_tb3_sim.gif)](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_tb3_sim.mp4)

**MoveIt:** `ros2 launch moveit_resources_panda_moveit_config demo.launch.py` with [this config](examples/moveit_panda/config.yaml).

[![ros_rerun_franka_moveit.mp4](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_franka_moveit.gif)](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_franka_moveit.mp4)

## Features

* **Config-driven:** one YAML file says which topics to show and how, and is checked when the bridge starts.
* **TF and URDF aware:** data is drawn in its TF frame. The robot model is loaded from its URDF and moves with TF or joint states.
* **rviz's common displays:** robot model, images, point clouds, laser scans, range sensors, markers, maps and costmaps, and more (see [Modules](#modules)).
* **Remote viewing:** any number of native or browser viewers can connect to a bridge running on a robot.
* **Extensible:** support a new message type with a small module class, in this package or in your own.

## Quick start

You need ROS 2 and [uv](https://docs.astral.sh/uv/). The bridge runs in its own Python environment, because Rerun needs NumPy 2 and ROS ships NumPy 1. uv creates that environment for you.

```bash
git clone https://github.com/justagist/ros_rerun_bridge.git && cd ros_rerun_bridge
source /opt/ros/jazzy/setup.bash
uv run ros-rerun-bridge --config examples/robot_arm/config.yaml
# In a second terminal, from the repo root with ROS sourced: publish a small arm's URDF and joint states
python3 examples/robot_arm/publisher.py
```

This opens the Rerun viewer, showing the arm moving and its joint positions plotted. The publisher only provides demo data: with a real robot, the bridge needs just a config listing its topics. [examples](examples) has more, each runnable the same way.

![robot_arm](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/robot_arm.gif)

See [INSTALLATION.md](docs/INSTALLATION.md) to make the `ros-rerun-bridge` command available everywhere, for other ways to install, and for why the bridge needs its own environment.

## Usage

```bash
ros-rerun-bridge --config <config.yaml> [options] [-- <ROS arguments>]
```

| Option | |
| --- | --- |
| *(no option)* | Open a Rerun viewer. |
| `--connect [--url <url>]` | Send to a Rerun viewer that is already running. |
| `--serve [--allow-origin <pattern>]` | Let viewers on other machines connect ([REMOTE_VIEWING.md](docs/REMOTE_VIEWING.md)). |
| `--save <file.rrd>` | Record to a file, to open later with `rerun <file.rrd>`. |
| `--stdout` | Stream to standard output, to pipe into `rerun -`. |
| `--headless` | Run without a viewer. |
| `--version` | Print the version. |

Arguments after `--` are passed to ROS, e.g. `-- --ros-args -p use_sim_time:=true`.

## Configuration

A config lists the topics to show. Each topic is handled by a module, which decides how it is drawn:

```yaml
global_frame: map            # the TF frame the 3D view is rendered in
modules:
  - name: tf
    module: tf
    topic: /tf
    entity_path: tf
  - name: scan
    module: laserscan
    topic: /scan
    entity_path: scan
    qos: { reliability: best_effort }   # most sensor drivers publish best effort
    params:
      mode: points
```

[CONFIGURATION.md](docs/CONFIGURATION.md) describes every field, and covers frames, QoS and saving a viewer layout. The [examples](examples) are a good starting point: each has a config, an optional viewer layout, and a publisher of demo data that only stands in for a real robot.

## Modules

Built-in modules cover rviz's common displays: `tf`, `robot_description`, `joint_states`, `image`, `compressed_image`, `camera_info`, `pointcloud2`, `laserscan`, `range`, `marker_array`, `occupancy_grid` and `particle_cloud`. The `scalars` module plots numeric fields of any message as time series, `bar_chart` draws an array field as bars, and `states` shows a field as states over time. See [MODULES.md](docs/MODULES.md) for what each shows and its parameters, and [EXTENDING.md](docs/EXTENDING.md) to add your own.

## Development

```bash
source /opt/ros/jazzy/setup.bash
uv run pytest            # tests (no running ROS nodes needed)
uv run ruff check        # lint
uv run ruff format       # format
```

Tests drive each module with real ROS messages and check what it logs to an in-memory Rerun recording. CI runs them in the `ros:jazzy-ros-base` image.

## License

[MIT](LICENSE)
