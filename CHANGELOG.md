# Changelog

All notable changes to this project are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html): before 1.0, a minor version may break configs or the module API.

## [0.11.0] - 2026-10-28

### Added

- `rosout` module, showing ROS log entries in Rerun's text log view, per logger.
- `diagnostics` module, showing each diagnostic status's level as states over time, and its messages as text logs.
- A logs and diagnostics example.
- The README covers recording, and viewing ROS bags through the bridge or in Rerun directly.

## [0.10.0] - 2026-10-28

### Added

- `pose` module, drawing a pose as axes, and its position covariance as an ellipsoid for `PoseWithCovarianceStamped` and `Odometry`.
- `pose_array` module, drawing an arrow per pose.
- `wrench` module, drawing force and torque arrows.
- A geometry example: a robot's pose with covariance, guesses at it, and a force-torque sensor.

### Changed

- A `module` path that does not name a `TopicToComponentModule` subclass is an error when the bridge starts.

## [0.9.0] - 2026-10-28

### Added

- `axis_length` and `frame_names` params for `tf` and `joint_states`, drawing each frame's axes and name, as rviz's TF display does. Each frame is drawn once, on its own entity under `frames`, so the viewer can show or hide each one.

### Changed

- The mobile robot and robot arm examples show their frames as named axes, and the MoveIt example its link frames as axes.

## [0.8.0] - 2026-10-28

### Added

- `depth_image` module, for 16UC1 and 32FC1 depth images, drawn as point clouds through the camera's pinhole.
- `compressed_depth_image` module, for compressedDepth images in PNG (16UC1) and RVL.

### Changed

- The camera example is now an RGB-D camera: its colour and aligned depth images show the same rippling surface.

## [0.7.0] - 2026-10-28

### Added

- `states` module, showing a field of any message type as discrete states over time, with labels and colours per value.
- A states example: a robot's mode and a warning light.

## [0.6.0] - 2026-10-28

### Added

- `bar_chart` module, drawing an array field of any message type as bars, e.g. joint efforts.

### Changed

- The robot arm example also shows its joint positions as bars.

## [0.5.0] - 2026-10-28

### Added

- `variance` and `unit` params for `scalars`: a value's uncertainty, drawn as a band, and its unit, using Rerun's measurements.

### Changed

- The range sensor example plots its readings with their uncertainty.
- The robot arm example's gripper fingers touch when closed.

## [0.4.0] - 2026-10-27

### Added

- Series style params for `scalars`: lines or points, names, colours, line width and interpolation (e.g. steps), and marker shape and size.
- `examples/`: runnable examples, each a config, a viewer layout and a publisher of made-up data, for a robot arm, a mobile robot, a camera, a point cloud, a range sensor, markers and plot styles.

### Changed

- The example configs moved from `example_configs/` to `examples/nav2_turtlebot3/` and `examples/moveit_panda/`.

### Fixed

- Colour params (`scalars`, `range`, `particle_cloud`) reject channels outside 0-255, which Rerun would otherwise wrap round.

## [0.3.0] - 2026-10-27

### Removed

- `odometry` module: `scalars` plots the same velocities, and any other field. Replace `module: odometry` with `module: scalars`, `msg_type: nav_msgs.msg.Odometry` and `params: { fields: [twist.twist.linear.x, twist.twist.angular.z] }`. The plots move from `<entity_path>/vel` and `<entity_path>/ang_vel` to `<entity_path>/twist/twist/linear/x` and `<entity_path>/twist/twist/angular/z`.

## [0.2.0] - 2026-10-27

### Added

- `scalars` module, plotting numeric fields of any message type as time series.

## [0.1.1] - 2026-10-27

### Fixed

- A marker changing type, or deleted and re-added with the same stamp, was not drawn.
- `TRIANGLE_LIST` markers were not drawn.
- A marker changing to an unsupported type, such as `MESH_RESOURCE`, kept showing its old shape.

## [0.1.0] - 2026-10-27

Requires Rerun 0.40 (pinned), and therefore NumPy 2: run the bridge in its own environment, e.g. with `uv run` (see `docs/INSTALLATION.md`).

### Added

- `tf` module, logging TF as relations between named frames. Every other module places its data in its message's `header.frame_id`, as rviz does.
- `compressed_image`, `marker_array`, `range` and `occupancy_grid` modules.
- `blueprint:` config option, to open the viewer with a saved layout.
- `--allow-origin` option, so web viewers on other machines can connect with `--serve`. With `--serve`, the bridge prints how to connect.
- `auto` point cloud colour mode, the new default: the `rgb` field if the cloud has one, otherwise no colour.
- Mimic joints follow the joint they mimic in `joint_states`.
- `intrinsics` parameter of `camera_info`, to show rectified images with the projection matrix `P`.
- `uv run` and `uv tool install` support, with the Rerun viewer found in the bridge's own environment.
- Test suite, CI, MIT licence, and documentation in `docs/`.

### Changed

- `robot_description` uses Rerun's URDF importer: link geometry is placed in each link's frame and moves with TF or `joint_states`. Collada meshes are scaled by their declared unit, and collision geometry is no longer logged.
- `image` and `camera_info` no longer need OpenCV, `cv_bridge` or `image_geometry`. Images are shown in 3D through the camera's pinhole.
- `qos` config entries are validated: invalid values and unknown keys are errors.
- The config, including its blueprint path, is validated before a viewer opens.
- Built-in modules are only imported when a config uses them, so a missing optional dependency only affects its module.
- `global_frame` sets the frame the default 3D view is rendered in.
- Errors while handling a message are logged with their type, and never stop the bridge.
- `--serve --headless` no longer opens a browser.
- Requires Python 3.12 or later.

### Removed

- `tf_paths` module parameter: configure the `tf` module instead.
- `link_scales` parameter of `robot_description`: Collada units are now applied automatically.
- `encoding_override` parameter of `image`: Rerun displays every supported encoding natively.
- The placeholder box logged at `<global_frame>/box`.

### Fixed

- Messages without a stamp were logged at the time of an unrelated earlier message; they are now logged at the time they are received.
- Topics were handled one at a time, despite the multi-threaded executor.
- Point cloud colours were misaligned when a cloud had NaN points, and had red and blue swapped.
- Laser scans were drawn as one zigzag line instead of one line per ray.
- `particle_cloud` failed in its default `points` mode.
- `CUBE_LIST` and `SPHERE_LIST` markers drew only their first shape.
- The bridge exited with an error, and could lose recorded data, on Ctrl+C.

## [0.0.1] - 2025-11-03

First version: a config-driven bridge with `image`, `camera_info`, `laserscan`, `pointcloud2`, `particle_cloud`, `odometry`, `robot_description` and `joint_states` modules.

[0.10.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/justagist/ros_rerun_bridge/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/justagist/ros_rerun_bridge/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/justagist/ros_rerun_bridge/compare/c6298dd...v0.1.0
[0.0.1]: https://github.com/justagist/ros_rerun_bridge/tree/c6298dd
