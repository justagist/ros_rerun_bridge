# Examples

Each folder holds what an example needs:

* `config.yaml`: the bridge config. This is the only file the bridge needs, and the part to copy for your own robot: point its topics at yours.
* `layout.rbl` (optional): the viewer layout and camera angle the config opens with. Without it, Rerun picks a layout itself.
* `publisher.py`: publishes made-up data for the demo, so it runs without a robot or simulator. On a real robot, your own nodes publish the data instead. The robot arm's `robot.urdf` is part of its demo data.

Some examples use modules that need extra ROS packages, e.g. `mobile_robot` and `point_cloud` ([INSTALLATION.md](../docs/INSTALLATION.md#ros-packages-for-some-modules)). From the repo root, with ROS sourced, run the bridge and the publisher in two terminals:

```bash
uv run ros-rerun-bridge --config examples/<name>/config.yaml
python3 examples/<name>/publisher.py
```

The publishers run with ROS's own Python, not the bridge's environment. They publish on common topics such as `/tf` and `/joint_states`, so run them away from a live robot, or in a ROS domain of their own (the same `export ROS_DOMAIN_ID=<n>` in both terminals).

The `layout.rbl` files are written by [layouts.py](layouts.py): to change a layout, edit it there and run `uv run python examples/layouts.py`. In your own config, drop the `blueprint:` line to let Rerun choose a layout, or point it at one you saved from the viewer.

## [robot_arm](robot_arm)

A small arm from its [URDF](robot_arm/robot.urdf), swinging and opening its gripper, with its link frames as named axes. The right finger follows the left one through a mimic joint. Its joint positions are plotted over time, and as bars at the time cursor. Modules: `robot_description`, `joint_states`, `scalars`, `bar_chart`.

![robot_arm](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/robot_arm.gif)

## [mobile_robot](mobile_robot)

A robot driving in a circle in a room: the room's map, the robot's frames as named axes, and its laser scan as rays to each hit, with a shadow behind the pillar. Its speed is plotted too. Modules: `tf`, `occupancy_grid`, `laserscan`, `scalars`.

![mobile_robot](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/mobile_robot.gif)

## [camera](camera)

An RGB-D camera looking at a rippling surface: its colour image in 2D and on the camera's image plane in 3D, and its aligned depth image in 2D and as a point cloud. Modules: `tf`, `camera_info`, `image`, `depth_image`.

![camera](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/camera.gif)

## [segmentation](segmentation)

A camera looking at a ball and a box moving over a floor, and a segmentation model's mask of them, drawn over the image in 2D and on the camera's image plane in 3D. The overlay shows where the model is wrong: its blocky edges, and the ball's shadowed side it misses. Hover over the mask to see a pixel's class. Modules: `tf`, `camera_info`, `image`, `segmentation_image`.

![segmentation](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/segmentation.gif)

## [point_cloud](point_cloud)

A rotating rainbow helix, coloured from the cloud's `rgb` field. Modules: `pointcloud2`.

![point_cloud](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/point_cloud.gif)

## [range_sensor](range_sensor)

A tilted range sensor's cone, and its readings plotted with a band showing their uncertainty. The cone disappears, and the plot has a gap, while it detects nothing. Modules: `tf`, `range`, `scalars`.

![range_sensor](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/range_sensor.gif)

## [markers](markers)

A marker changing type every 2 s, beside a fixed cube. Modules: `marker_array`.

![markers](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/markers.gif)

## [geometry](geometry)

A robot circling with its pose as axes and its position covariance as an ellipsoid, a cloud of guesses at its pose, and a force-torque sensor's force and torque arrows on a tool. Modules: `tf`, `pose`, `pose_array`, `wrench`.

![geometry](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/geometry.gif)

## [states](states)

A robot's mode, cycling through idle, moving and charging, and a warning light that blinks while it moves, as coloured states over time. Modules: `states`.

![states](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/states.gif)

## [logs_and_diagnostics](logs_and_diagnostics)

An arm and a planner node logging through a pick-and-place cycle, and the arm's diagnostics: its motor heating up to a warning and an error, its gripper going stale, and its battery running low. Modules: `rosout`, `diagnostics`.

![logs_and_diagnostics](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/logs_and_diagnostics.gif)

## [series_styles](series_styles)

The same values as lines and as named points, and a mode drawn in red steps. Modules: `scalars`.

![series_styles](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/series_styles.gif)

## Configs for other demos

These have no publisher: run the bridge with them alongside the demo.

* [nav2_turtlebot3](nav2_turtlebot3/config.yaml): the [Nav2 getting started demo](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/).
* [moveit_panda](moveit_panda/config.yaml): the MoveIt demo, `ros2 launch moveit_resources_panda_moveit_config demo.launch.py`.
