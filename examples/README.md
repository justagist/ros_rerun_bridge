# Examples

Each folder holds a bridge config. Most also have a `publisher.py` that publishes made-up data, so the example runs without a robot or simulator. From the repo root, with ROS sourced, run the bridge and the publisher in two terminals:

```bash
uv run ros-rerun-bridge --config examples/<name>/config.yaml
python3 examples/<name>/publisher.py
```

The publishers run with ROS's own Python, not the bridge's environment. They publish on common topics such as `/tf` and `/joint_states`, so run them away from a live robot, or in a ROS domain of their own (the same `export ROS_DOMAIN_ID=<n>` in both terminals).

## [robot_arm](robot_arm)

A small arm from its [URDF](robot_arm/robot.urdf), swinging and opening its gripper. The right finger follows the left one through a mimic joint. Its joint positions are plotted. Modules: `robot_description`, `joint_states`, `scalars`.

![robot_arm](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/robot_arm.gif?v=2)

## [mobile_robot](mobile_robot)

A robot driving in a circle in a room: the room's map, and the robot's laser scan as rays to each hit, with a shadow behind the pillar. Its speed is plotted too. Modules: `tf`, `occupancy_grid`, `laserscan`, `scalars`.

![mobile_robot](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/mobile_robot.gif)

## [camera](camera)

Moving colour bands, in a 2D view and on the camera's image plane in 3D. Modules: `tf`, `camera_info`, `image`.

![camera](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/camera.gif)

## [point_cloud](point_cloud)

A rotating rainbow helix, coloured from the cloud's `rgb` field. Modules: `pointcloud2`.

![point_cloud](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/point_cloud.gif)

## [range_sensor](range_sensor)

A tilted range sensor's cone, and its readings plotted. The cone disappears, and the plot has a gap, while it detects nothing. Modules: `tf`, `range`, `scalars`.

![range_sensor](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/range_sensor.gif)

## [markers](markers)

A marker changing type every 2 s, beside a fixed cube. Modules: `marker_array`.

![markers](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/markers.gif)

## [series_styles](series_styles)

The same values as lines and as named points, and a mode drawn in red steps. Modules: `scalars`.

![series_styles](https://media.githubusercontent.com/media/justagist/_assets/main/ros_rerun_bridge/examples/series_styles.gif)

## Configs for other demos

These have no publisher: run the bridge with them alongside the demo.

* [nav2_turtlebot3](nav2_turtlebot3/config.yaml): the [Nav2 getting started demo](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/).
* [moveit_panda](moveit_panda/config.yaml): the MoveIt demo, `ros2 launch moveit_resources_panda_moveit_config demo.launch.py`.
