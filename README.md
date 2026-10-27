# ros-rerun-bridge

[![CI](https://github.com/justagist/ros_rerun_bridge/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/justagist/ros_rerun_bridge/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Rerun 0.40](https://img.shields.io/badge/Rerun-0.40-0B8CE9.svg)](https://rerun.io)
[![uv](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json)](https://github.com/astral-sh/uv)

Show live ROS 2 data in the [Rerun](https://rerun.io) viewer, as you would in rviz. You list the topics to show in a YAML config; the bridge subscribes to them and logs each message to Rerun, placed in its TF frame.

> **Status:** I created this repository mainly to get familiar with Rerun and to use it in my own ROS projects. It is under active development, so features, interfaces and behaviour may change. Tested with Rerun 0.40.0 and ROS 2 Jazzy.

## Demos

**Nav2:** the [Nav2 getting started demo](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/) with [turtlebot3_nav2.config.yaml](example_configs/turtlebot3_nav2.config.yaml).

[![ros_rerun_tb3_sim.mp4](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_tb3_sim.gif)](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_tb3_sim.mp4)

**MoveIt:** `ros2 launch moveit_resources_panda_moveit_config demo.launch.py` with [robot_joint_states.config.yaml](example_configs/robot_joint_states.config.yaml).

[![ros_rerun_franka_moveit.mp4](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_franka_moveit.gif)](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_franka_moveit.mp4)

## Features

* **Config-driven:** one YAML file lists the topics, their QoS, where they appear in Rerun, and per-module parameters, validated on start.
* **TF and URDF aware:** data appears in its TF frame, and the robot model, loaded from its URDF, moves with TF or joint states.
* **rviz's common displays:** robot model, images, point clouds, laser scans, range sensors, markers, maps and costmaps, and more (see [modules](#modules)).
* **Remote viewing:** any number of native or browser viewers can connect to a bridge running on a robot.
* **Extensible:** a new message type is a small module class, built in or loaded from your own package.

## Quick start

Needs a ROS 2 installation and [uv](https://docs.astral.sh/uv/). The bridge runs in its own Python environment, as Rerun needs NumPy 2 while ROS ships NumPy 1; uv sets that up.

```bash
git clone https://github.com/justagist/ros_rerun_bridge.git && cd ros_rerun_bridge
source /opt/ros/jazzy/setup.bash
uv run ros-rerun-bridge --config example_configs/robot_joint_states.config.yaml
```

This opens the Rerun viewer and shows any robot publishing `/robot_description` and `/joint_states`, such as the MoveIt demo above.

To have the `ros-rerun-bridge` command available everywhere:

```bash
uv tool install --python-preference only-system -e .
```

See [INSTALLATION.md](docs/INSTALLATION.md) for other ways to install, and why the bridge needs its own environment.

## Usage

```bash
ros-rerun-bridge --config <config.yaml> [options] [-- <ROS arguments>]
```

| Option | |
| --- | --- |
| *(none)* | Open a Rerun viewer. |
| `--connect [--url <url>]` | Send to a Rerun viewer that is already running. |
| `--serve [--allow-origin <pattern>]` | Host servers for viewers on other machines ([REMOTE_VIEWING.md](docs/REMOTE_VIEWING.md)). |
| `--save <file.rrd>` | Record to a file, to open later with `rerun <file.rrd>`. |
| `--stdout` | Stream to standard output, to pipe into `rerun -`. |
| `--headless` | Run without a viewer. |
| `--version` | Print the version. |

Arguments after `--` are passed to ROS, e.g. `-- --ros-args -p use_sim_time:=true`.

## Configuration

A config lists the topics to show, each handled by a module:

```yaml
global_frame: map            # the TF frame the 3D view is rendered in
modules:
  - { name: tf, module: tf, topic: /tf, entity_path: tf }
  - name: scan
    module: laserscan
    topic: /scan
    entity_path: scan
    params: { mode: points }
```

See [CONFIGURATION.md](docs/CONFIGURATION.md) for every field, frames, QoS and saving a viewer layout. Two example configs are in [example_configs](example_configs):

* [robot_joint_states.config.yaml](example_configs/robot_joint_states.config.yaml): a robot from its URDF and joint states. Works as is with the MoveIt demos.
* [turtlebot3_nav2.config.yaml](example_configs/turtlebot3_nav2.config.yaml): TF, robot, camera, point cloud, laser scan, map, costmap, particles and odometry from the Nav2 demo.

## Modules

| Module | Message | Shows |
| --- | --- | --- |
| `tf` | `tf2_msgs/TFMessage` | Links between TF frames |
| `robot_description` | `std_msgs/String` | The robot model from its URDF |
| `joint_states` | `sensor_msgs/JointState` | Robot joints, without TF |
| `image`, `compressed_image` | `sensor_msgs/Image`, `CompressedImage` | Camera images |
| `camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics, to show images in 3D |
| `pointcloud2` | `sensor_msgs/PointCloud2` | Point clouds |
| `laserscan` | `sensor_msgs/LaserScan` | Laser scans |
| `range` | `sensor_msgs/Range` | Range sensor readings |
| `marker_array` | `visualization_msgs/MarkerArray`, `Marker` | rviz markers |
| `occupancy_grid` | `nav_msgs/OccupancyGrid` | Maps and costmaps |
| `odometry` | `nav_msgs/Odometry` | Velocities as plots |
| `particle_cloud` | `nav2_msgs/ParticleCloud` | Localisation particles |

See [MODULES.md](docs/MODULES.md) for what each shows and its parameters, and [EXTENDING.md](docs/EXTENDING.md) to add your own.

## Development

```bash
source /opt/ros/jazzy/setup.bash
uv run pytest            # tests: no running ROS graph needed
uv run ruff check        # lint
uv run ruff format       # format
```

Tests drive each module with real ROS messages and check what it logs to an in-memory Rerun recording. CI runs them in the `ros:jazzy-ros-base` image.

## License

[MIT](LICENSE)
