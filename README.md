
# ros-rerun-bridge

A **config-driven, modular ROS 2 → Rerun** bridge. You describe what to subscribe to and how to visualize it in a YAML file; the bridge loads a set of lightweight **modules** (one topic → one Rerun component) and streams everything into the Rerun viewer.

> 🚧 Project Status: Active Development
> 
> **Note:** This repository is under active development. Features, interfaces, and behavior may change, and bugs or incomplete functionality could be present.

---

## Highlights

* **Modular by design:** add/remove per-topic handlers as plugins.
* **Config-first:** ship a single YAML that declares topics, QoS, entity paths, and per-module parameters.
* **Zero-copy-ish visuals:** modules translate ROS messages to Rerun primitives directly.
* **TF & URDF aware:** optional helpers to visualize frames and robot geometry; `joint_states` can animate URDF.
* **Module-specific params:** Each module can have module-specific params. Module params are validated with Pydantic (v2) for stricter typing.

---

## Installation (dev)

```bash
# From the repo root
pip install -e .

# Make sure your ROS 2 distro is installed & sourced (e.g. Humble):
#   source /opt/ros/humble/setup.bash
```

> Python deps are declared in `pyproject.toml` (e.g. `pydantic`, `pyyaml`, `opencv-python`, `trimesh`, `yourdfpy`). ROS-supplied Python packages (e.g. message types) come from your sourced ROS environment.

---

## Running

The package exposes a console script:

```bash
ros-rerun-bridge --config path/to/config.yaml [RERUN_ARGS …] -- [RCLPY_ARGS …]
```

* `--config` points to your YAML (schema below).
* **RERUN_ARGS** are forwarded to `rerun.script_setup(...)` (e.g. `--spawn`, `--connect`, `--save` etc.).
* After a literal `--`, any **RCLPY_ARGS** are passed through to rclpy/ROS 2.
* `--version` prints the package version and exits.

Example:

```bash
# Spawn a Rerun viewer and start the bridge.
ros-rerun-bridge --config example_configs/turtlebot3_nav2.config.yaml --spawn

# Or connect to an already-running viewer
ros-rerun-bridge --config example_configs/robot_joint_states.config.yaml --connect 127.0.0.1:9876
```

---

## Configuration schema

At the top level:

```yaml
global_frame: map   # (optional) default TF global frame for helpers
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

### Module resolution

* Use a **registry key** (e.g. `image`, `pointcloud2`) to load a built‑in module, **or**
* Provide a **Python path** to a class (`package.module:ClassName` or `package.module.ClassName`).

### Shared helpers available to modules

* **TF helper** (`node.tf`): modules that include `tf_paths` in their params can log transforms per message timestamp.
* **URDF kinematics** (`context.urdf_kinematics`): exposed once the `urdf` module loads; used by `joint_states` to animate links.

> Internally, params inherit from a strict `BaseModel` so unknown fields error out instead of being ignored.

---

## Built‑in modules

Below is the current set of bundled modules and their notable params.

> All modules accept `entity_path` and (unless otherwise stated) support optional `params.tf_paths: [ {path, child_frame, parent_frame}, … ]` to log transforms alongside the primary visualization.

### `image`  — `sensor_msgs/Image`

* Converts via `cv_bridge` and logs `rr.Image`.
* **Params**

  * `encoding_override: <string>` – force a cv_bridge target encoding.

### `camera_info`  — `sensor_msgs/CameraInfo`

* Uses `image_geometry.PinholeCameraModel` to log intrinsics to Rerun (`rr.Pinhole`), aligned with your `entity_path`.

### `laserscan`  — `sensor_msgs/LaserScan`

* Converts a scan to points or lines using `laser_geometry`/`sensor_msgs_py.point_cloud2`.
* **Params**

  * `render_mode: points | lines` (default `points`)
  * `min_range` / `max_range` – clamp distances
  * `decimation: int` – sample every Nth ray
  * `radius: float` – line radius when `render_mode=lines`

### `pointcloud2`  — `sensor_msgs/PointCloud2`

* Logs `rr.Points3D` with optional RGB extraction.
* **Params**

  * `colour_mode: rgb_packed | fields | none` (default `rgb_packed`)
  * `colour_fields: [r, g, b]` – when `colour_mode=fields`

### `odometry`  — `nav_msgs/Odometry` (optional dep)

* Logs scalar time‑series for linear/angular velocities and supports TF path logging via `tf_paths`.

### `urdf`  — `std_msgs/String` (XML from `/robot_description`)

* Parses the URDF, logs link visuals, and exposes kinematics in `context.urdf_kinematics`.
* **Params**

  * `link_scales: { <link_name>: <scale> }` – per‑link scale factors for visuals.

### `joint_states`  — `sensor_msgs/JointState`

* Updates joint positions in the URDF scene (requires `urdf` module to be loaded first).

---

## Examples

The repo includes two sample configs in `example_configs/`:

* `robot_joint_states.config.yaml` – load URDF once, then animate with `/joint_states`. This should work with any robot publishing /robot_description and /joint_states. 
* `turtlebot3_nav2.config.yaml` – camera, laser scan, odometry, point clouds, etc. This can be used with the example from nav2 starter doc: https://docs.nav2.org/getting_started/index.html.

Run either as:

```bash
ros-rerun-bridge --config example_configs/turtlebot3_nav2.config.yaml --spawn
```

---
