# Changelog

All notable changes to this project are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html): before 1.0, a minor version may break configs or the module API.

## [Unreleased]

## [0.1.0] - 2026-10-27

Requires Rerun 0.40 (pinned), and therefore NumPy 2: run the bridge in its own environment, e.g. with `uv run` (see `docs/INSTALLATION.md`).

### Added

- `tf` module, logging TF as relations between named frames. Every other module places its data in its message's `header.frame_id`, as rviz does.
- `compressed_image`, `marker_array`, `range` and `occupancy_grid` modules.
- `blueprint:` config option, to open the viewer with a saved layout.
- `--allow-origin` option, so web viewers on other machines can connect with `--serve`. With `--serve`, the bridge prints how to connect.
- `auto` point cloud colour mode, the new default: the `rgb` field if the cloud has one, otherwise no colour.
- Mimic joints follow the joint they mimic in `joint_states`.
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

[Unreleased]: https://github.com/justagist/ros_rerun_bridge/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/justagist/ros_rerun_bridge/compare/c6298dd...v0.1.0
[0.0.1]: https://github.com/justagist/ros_rerun_bridge/tree/c6298dd
