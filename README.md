
# ros-rerun-bridge

A **config-driven, modular ROS 2 -> Rerun** bridge. You describe what to subscribe to and how to visualize it in a YAML file; the bridge loads a set of lightweight **modules** (one topic -> one Rerun component) and streams everything into the Rerun viewer.

> 🚧 In Active Development
> 
> **Note:** This repository is created primarily so that I can get familiar with rerun and to try using it for some of my personal ROS projects. It is under active development; so features, interfaces, and behavior may change, and bugs or incomplete functionality could be present.

**Tested with Rerun sdk v0.40.0 and ROS Jazzy.**

---

# Demos

### Nav2 Demo

This uses the [turtlebot3_nav2.config.yaml](example_configs/turtlebot3_nav2.config.yaml) config file and works directly with the [nav2 starter demo](https://docs.nav2.org/getting_started/index.html).

[![ros_rerun_tb3_sim.mp4](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_tb3_sim.gif)](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_tb3_sim.mp4)

### Moveit Demo

This uses the [robot_joint_states.config.yaml](example_configs/robot_joint_states.config.yaml) config file with `ros2 launch moveit_resources_panda_moveit_config demo.launch.py`, and works with any robot that publishes robot description and joint states (set `global_frame` to its root link if nothing publishes a `world` frame).

[![ros_rerun_franka_moveit.mp4](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_franka_moveit.gif)](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/ros_rerun_franka_moveit.mp4)


---

## Highlights

* **Modular by design:** add/remove per-topic handlers as plugins.
* **Config-first:** ship a single YAML that declares topics, QoS, entity paths, and per-module parameters.
* **Zero-copy-ish visuals:** modules translate ROS messages to Rerun primitives directly.
* **TF & URDF aware:** data is placed in the TF frame named in each message's header, as in rviz; the robot model is loaded with Rerun's URDF importer and moves with TF (or `joint_states`).
* **Module-specific params:** Each module can have module-specific params. Module params are validated with Pydantic (v2) for stricter typing.

---

## Installation

Rerun needs NumPy 2, while ROS distros ship NumPy 1, so the bridge runs in its own virtual environment. ROS-supplied Python packages (`rclpy`, message types) come from your sourced ROS environment through `PYTHONPATH`.

* Do not install it into the system Python: upgrading NumPy there breaks other ROS nodes (e.g. OpenCV, `cv_bridge`).
* Do not create the environment with system site-packages: the distro's NumPy 1 builds of packages such as pandas then break Rerun.
* The environment must use the Python your ROS distro is built for (Python 3.12 on Jazzy), as `rclpy` is compiled for it.

[uv](https://docs.astral.sh/uv/) handles all of this. The repo configures it to use only system Pythons, never a uv-managed one:

```bash
source /opt/ros/jazzy/setup.bash
# From the repo root: creates .venv on first run, then runs the console script
uv run ros-rerun-bridge --config example_configs/robot_joint_states.config.yaml
```

Without cloning, `uvx` does not read the repo's uv settings, so ask for a system Python explicitly:

```bash
uvx --python-preference only-system --from git+https://github.com/justagist/ros_rerun_bridge ros-rerun-bridge --config <config.yaml>
```

To make `ros-rerun-bridge` available everywhere, install it as a uv tool. It gets its own environment and the command goes in `~/.local/bin` (run `uv tool update-shell` if that is not on your `PATH`). Like `uvx`, this does not read the repo's uv settings:

```bash
# From a clone, editable: code changes apply without reinstalling (add --reinstall after dependency changes)
uv tool install --python-preference only-system -e .
# Or straight from GitHub
uv tool install --python-preference only-system git+https://github.com/justagist/ros_rerun_bridge

source /opt/ros/jazzy/setup.bash
ros-rerun-bridge --config <config.yaml>

# To remove it
uv tool uninstall ros-rerun-bridge
```

Or with pip, in a plain venv created from the distro's Python (on Ubuntu this needs the `python3-venv` package):

```bash
/usr/bin/python3 -m venv .venv && .venv/bin/pip install -e .
.venv/bin/ros-rerun-bridge --config <config.yaml>
```

---

## Running

The package exposes a console script:

```bash
ros-rerun-bridge --config path/to/config.yaml [RERUN_ARGS …] -- [RCLPY_ARGS …]
```

* `--config` points to your YAML (schema below).
* **RERUN_ARGS** are forwarded to `rerun.script_setup(...)`. Without any, the bridge spawns a native Rerun viewer. Others: `--connect` (to a running viewer, default address, or `--url`), `--serve` (servers for remote viewers, see below), `--save <file.rrd>`, `--stdout`, `--headless`.
* `--allow-origin <pattern>` (repeatable) lets web viewers served from other origins connect with `--serve`.
* After a literal `--`, any **RCLPY_ARGS** are passed through to rclpy/ROS 2.
* `--version` prints the package version and exits.

Example:

```bash
# Spawn a Rerun viewer and start the bridge.
uv run ros-rerun-bridge --config example_configs/turtlebot3_nav2.config.yaml

# Or connect to an already-running viewer
uv run ros-rerun-bridge --config example_configs/robot_joint_states.config.yaml --connect
```

### Viewing from other machines

With `--serve`, the bridge hosts a gRPC server (port 9876) and a web viewer (port 9090), both on all interfaces, and prints how to connect. Any number of viewers can connect and disconnect at any time; one joining late gets the recent history (up to 1 GiB).

* **Native viewer:** `rerun rerun+http://<host>:9876/proxy`, using Rerun 0.40.0 to match the bridge (e.g. `uvx --from rerun-sdk==0.40.0 rerun ...`). On the bridge's own machine add `--port auto`, as the bridge already uses 9876.
* **Browser:** `http://<host>:9090/?url=rerun%2Bhttp%3A%2F%2F<host>%3A9876%2Fproxy`. The page only connects when given the server in `url`. The server only accepts web viewers served from `localhost` or rerun.io, so a browser reaching the bridge by any other address needs that origin allowed, e.g. `--allow-origin 'http://192.168.1.10:*'`.

Neither connection is authenticated or encrypted: anyone who can reach the ports, or whose origin is allowed, can read the stream. Off a trusted network, use an SSH tunnel (`ssh -L 9876:localhost:9876 -L 9090:localhost:9090 <host>`) and `localhost` instead. In Docker, publish both ports.

---

## Configuration schema

At the top level:

```yaml
global_frame: map   # (optional) TF frame the default 3D view is rendered in
blueprint: layout.rbl  # (optional) viewer layout to open with, relative to this file (see below)
modules:            # list of module specs (loaded in order)
  - name: <string>          # required, human-friendly
    module: <id-or-path>    # required, registry key OR python path (see below)
    topic: <ros-topic>      # required, ROS topic to subscribe
    entity_path: <rerun/entity>  # required, where to log in Rerun
    msg_type: <pkg/msg>     # optional override; module has a default
    qos:                    # optional QoS (all fields optional)
      depth: 10
      reliability: best_effort | reliable
      durability: volatile | transient_local
      history: keep_last | keep_all
    params: {}              # optional, module-specific settings
```

### Viewer layout

Without a `blueprint`, Rerun picks a layout from the data. To keep one, arrange the viewer, save it with **Save blueprint…** in the viewer's file menu and point `blueprint` at the `.rbl` file. It applies whichever application it was saved from. Layouts can also be [written in Python](https://rerun.io/docs/howto/visualization/build-a-blueprint-programmatically) and saved with `Blueprint.save()`.

### Module resolution

* Use a **registry key** (e.g. `image`, `pointcloud2`) to load a built‑in module, **or**
* Provide a **Python path** to a class (`package.module:ClassName` or `package.module.ClassName`).

### Shared helpers available to modules

* **Frame-aware logging** (`self.log(..., frame_id=...)`): logs to the module's entity, placed in the named TF frame. TF itself comes from the `tf` module.
* **Robot model** (`context.urdf_tree`): Rerun's `UrdfTree`, exposed once the `robot_description` module loads; used by `joint_states` to move joints.

> Internally, params inherit from a strict `BaseModel` so unknown fields error out instead of being ignored.

---

## Built‑in modules

Below is the current set of bundled modules and their notable params. See params in their corresponding src file for full details.

> All modules accept `entity_path`. Modules for stamped messages place their data in the message's `header.frame_id`, so configure the `tf` module for anything to appear in the 3D view. `global_frame` sets the frame the default 3D view is rendered in.

### `tf`  -- `tf2_msgs/TFMessage`

* Logs each transform as a `rr.Transform3D` between named frames (`header.frame_id` -> `child_frame_id`).
* Use one module for `/tf` and another for `/tf_static`. Rerun requires a frame relation to always be logged on the same entity, so give them different entities, and log `/tf_static` on `tf_static`, the entity Rerun's URDF importer also logs the robot's joints on.
* **Params**

  * `static: bool` -- log transforms as static, valid at all times (default `false`). Set for `/tf_static`, with `durability: transient_local` QoS.

### `image`  -- `sensor_msgs/Image`

* Logs `rr.Image` in the frame `<frame_id>_image_plane`, where the `camera_info` pinhole shows it in 3D.
* Supports the RGB(A), BGR(A) and mono encodings at 8 and 16 bits, `8UC1`, `16UC1` and `32FC1`, Bayer 8-bit and `yuv422_yuy2`. Rerun displays them natively, so no conversion is needed.

### `compressed_image`  -- `sensor_msgs/CompressedImage`

* Logs JPEG and PNG images as `rr.EncodedImage`, for the viewer to decode, in the frame `<frame_id>_image_plane` like `image`.
* `compressedDepth` images are not supported.

### `camera_info`  -- `sensor_msgs/CameraInfo`

* Logs the intrinsics (`K`) as `rr.Pinhole` from the camera frame to its image plane frame, `<frame_id>_image_plane`.

### `laserscan`  -- `sensor_msgs/LaserScan`

* Converts a scan to points or lines using `laser_geometry`/`sensor_msgs_py.point_cloud2`.
* **Params**

  * `mode: points | lines` (default `points`)
  * `origin_scale`  -- Scale factor for the laser origin when rendering lines (default: 0.3 m) 
  * `radius: float` -- line radius when `render_mode=lines`

### `pointcloud2`  -- `sensor_msgs/PointCloud2`

* Logs `rr.Points3D` with optional RGB extraction.
* **Params**

  * `colour_mode: rgb_packed | fields | none` (default `rgb_packed`)
  * `colour_fields: [r, g, b]` -- when `colour_mode=fields`

### `particle_cloud`  -- `nav2_msgs/ParticleCloud`

* Visualizes a particle cloud as either points or arrows, depending on configuration.
* **Params**

  * `mode: "points" | "arrows"` -- Visualization mode. `'points'` uses `rr.Points3D`, `'arrows'` uses `rr.Arrows3D`.
  * `radius: float` -- Base radius for points or arrows (default: 0.0015).
  * `colour: [r, g, b, a]` -- Base RGBA colour (default: `[255, 255, 255, 255]`).
  * `colour_by_weight: bool` -- Whether to modulate colour alpha by particle weight.
  * `w_min` / `w_max`: Optional normalization bounds for weights.
  * `arrow_len: float` -- Base arrow length (default: 0.05).
  * `arrow_axis: "x" | "y" | "z"` -- Which body axis defines heading for arrows (default: x).

### `odometry`  -- `nav_msgs/Odometry` (optional dep)

* Logs scalar time‑series for linear/angular velocities. The robot's pose comes from TF.

### `marker_array`  -- `visualization_msgs/MarkerArray`

* Logs markers as rviz shows them, each on `<entity_path>/<ns>/<id>` and posed in its header frame. For a single `visualization_msgs/Marker` topic, set `msg_type: visualization_msgs.msg.Marker`.
* Supports every marker type except `MESH_RESOURCE`, the `ADD`, `DELETE` and `DELETEALL` actions (`DELETEALL` clears only the message's namespace if it has one, as in rviz), and lifetimes.
* Text markers are drawn as labels on a point, as Rerun has no 3D text. Markers always move with their frame, as if `frame_locked` were set.

### `range`  -- `sensor_msgs/Range`

* Draws a reading as rviz does: a cone along the sensor frame's x axis, with the measured range as its length and the field of view as its angle. Readings outside `[min_range, max_range]`, including `±inf` "no detection" readings, clear the cone.
* **Params**

  * `colour: [r, g, b, a]` (default white) and `radius: float` -- line radius in metres (default `0.001`).

### `occupancy_grid`  -- `nav_msgs/OccupancyGrid`

* Logs maps and costmaps as `rr.GridMap` in the message's frame, coloured as in rviz. `/map` is latched, so give it `durability: transient_local` QoS.
* **Params**

  * `colormap: RvizMap | RvizCostmap | Costmap | Grayscale` (default `RvizMap`).
  * `opacity: float` (default `1.0`) and `draw_order: float` -- e.g. to draw a semi-transparent costmap over the map.

### `robot_description`  -- `std_msgs/String` (XML from `/robot_description`)

* Loads the URDF from the /robot_description topic with Rerun's URDF importer and logs each link's visual geometry in the link's frame. `package://` mesh paths are resolved through the sourced ROS environment. Collision geometry is not logged.
* Scales Collada (`.dae`) visual meshes by the unit they declare (`<unit meter="...">`), as rviz does: Rerun's importer ignores it, so e.g. a mesh authored in inches would otherwise render ~40x too large.
* Logs the joints at zero as static defaults on `tf_static`, so the robot is assembled before TF or joint states arrive.
* Exposes the model in `context.urdf_tree` for other modules.

### `joint_states`  -- `sensor_msgs/JointState`

* Moves the robot model's joints from the published /joint_states topic (requires the `robot_description` module).
* Not needed where `robot_state_publisher` already puts the joints on `/tf`.
* Log it on the `tf` module's entity (`entity_path: tf`). Rerun requires a frame relation to always be logged on the same entity, so this lets the same joint arrive on both `/tf` and `/joint_states` (the latest transform wins, as in tf2), and lets `joint_states` move the arm while `/tf` carries other frames, e.g. a camera.

---

## Examples

The repo includes two sample configs in `example_configs/`:

* `robot_joint_states.config.yaml` -- load URDF once, then animate with `/joint_states`. Works as is with the MoveIt demos, and with any robot publishing /robot_description and /joint_states (set `global_frame` to its root link if nothing publishes a `world` frame). 
* `turtlebot3_nav2.config.yaml` -- camera, laser scan, odometry, point clouds, etc. This can be used with the example from nav2 starter doc: https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/#quickstart.

Run either as:

```bash
uv run ros-rerun-bridge --config example_configs/turtlebot3_nav2.config.yaml
```

---
