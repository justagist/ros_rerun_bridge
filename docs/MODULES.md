# Built-in modules

Each module subscribes to one topic and logs its messages under the module's `entity_path`. Select one with `module: <name>` in the config ([CONFIGURATION.md](CONFIGURATION.md)); set its parameters under `params`. Unknown parameters are an error.

Modules for stamped messages place their data in the TF frame named in its header and log it at its stamp; see [Frames](CONFIGURATION.md#frames) and [Timeline](CONFIGURATION.md#timeline).

| Module | Message | Shows |
| --- | --- | --- |
| [`tf`](#tf) | `tf2_msgs/TFMessage` | Links between TF frames |
| [`robot_description`](#robot_description) | `std_msgs/String` | The robot model from its URDF |
| [`joint_states`](#joint_states) | `sensor_msgs/JointState` | Robot joints, without TF |
| [`image`](#image) | `sensor_msgs/Image` | Camera images |
| [`compressed_image`](#compressed_image) | `sensor_msgs/CompressedImage` | JPEG and PNG camera images |
| [`camera_info`](#camera_info) | `sensor_msgs/CameraInfo` | Camera intrinsics, to show images in 3D |
| [`pointcloud2`](#pointcloud2) | `sensor_msgs/PointCloud2` | Point clouds |
| [`laserscan`](#laserscan) | `sensor_msgs/LaserScan` | Laser scans |
| [`range`](#range) | `sensor_msgs/Range` | Range sensor readings as cones |
| [`marker_array`](#marker_array) | `visualization_msgs/MarkerArray` or `Marker` | rviz markers |
| [`occupancy_grid`](#occupancy_grid) | `nav_msgs/OccupancyGrid` | Maps and costmaps |
| [`odometry`](#odometry) | `nav_msgs/Odometry` | Velocities as plots |
| [`particle_cloud`](#particle_cloud) | `nav2_msgs/ParticleCloud` | Localisation particles |
| [`scalars`](#scalars) | Any, set with `msg_type` | Numeric fields as plots |

## `tf`

Logs each transform as a link between named frames (`header.frame_id` to `child_frame_id`).

Use one module for `/tf` and another for `/tf_static`, on different entities: Rerun requires a link between two frames to change over time on one entity only (a static value on another entity is allowed, as a default). Log `/tf_static` on `tf_static`, the entity the `robot_description` module also logs the robot's joint defaults on.

```yaml
- { name: tf, module: tf, topic: /tf, entity_path: tf }
- name: tf_static
  module: tf
  topic: /tf_static
  entity_path: tf_static
  qos: { depth: 10, durability: transient_local }
  params: { static: true }
```

| Param | Default | |
| --- | --- | --- |
| `static` | `false` | Log transforms as valid at all times. Set for `/tf_static`. |

## `robot_description`

Loads the URDF from `/robot_description` with Rerun's URDF importer and logs each link's visual geometry in the link's frame, so the robot moves with TF or with [`joint_states`](#joint_states). Give it `durability: transient_local` QoS, as the topic is latched.

* `package://` mesh paths are resolved through the sourced ROS environment.
* Collision geometry is not logged.
* Collada (`.dae`) meshes are scaled by the unit they declare (`<unit meter="...">`), as rviz does. Rerun's importer ignores it, so a mesh authored in inches would otherwise render about 40 times too large.
* The joints at zero are logged as static defaults on `tf_static`, so the robot is assembled before TF or joint states arrive.
* The model is shared with other modules as `context.urdf_tree`.

## `joint_states`

Moves the robot model's joints from `/joint_states`. Needs the [`robot_description`](#robot_description) module. Joint positions are shown as reported, even outside the URDF's limits. Mimic joints follow the joint they mimic, as with `robot_state_publisher`, unless the message gives their position.

Not needed where `robot_state_publisher` already puts the joints on `/tf`. Log it on the `tf` module's entity (`entity_path: tf`): the same joint can then arrive on both `/tf` and `/joint_states` (the latest wins, as in tf2), and `/tf` can carry other frames, such as a camera's.

## `image`

Logs images in the frame `<frame_id>_image_plane`, where the [`camera_info`](#camera_info) module's pinhole shows them in 3D.

Supports the RGB(A), BGR(A) and mono encodings at 8 and 16 bits, `8UC1`, `16UC1`, `32FC1`, 8-bit Bayer and `yuv422_yuy2`. Rerun displays them natively, so nothing is converted.

## `compressed_image`

Logs JPEG and PNG images as they are, for the viewer to decode, in the frame `<frame_id>_image_plane` like [`image`](#image). `compressedDepth` images are not supported.

## `camera_info`

Logs the camera's intrinsics as a pinhole linking the camera frame to its image plane frame, `<frame_id>_image_plane`, adjusted for binning and region of interest. As in rviz, `frame_id` should be the camera's optical frame (`x` right, `y` down, `z` forward); another frame shows the image plane rotated.

| Param | Default | |
| --- | --- | --- |
| `intrinsics` | `k` | Which intrinsics match the images shown: `k` for raw images (e.g. `image_raw`), `p` (its left 3x3) for rectified ones (e.g. `image_rect`). |

## `pointcloud2`

Logs points with their colours. Points with a NaN coordinate are dropped.

| Param | Default | |
| --- | --- | --- |
| `colour_mode` | `auto` | `auto`: the `rgb` field if the cloud has one, otherwise no colour. `rgb_packed`: colour packed in a field named `rgb`, as PCL does. `fields`: separate channels. `none`: no colour. |
| `colour_fields` | `[r, g, b]` | Channel fields, for `colour_mode: fields`. |

## `laserscan`

Logs a scan as rays from the scanner, or as points.

| Param | Default | |
| --- | --- | --- |
| `mode` | `lines` | `lines` or `points`. |
| `origin_scale` | `0.3` | Distance from the scanner where rays start, in metres. |
| `radius` | `0.0025` | Ray line radius, in metres. |

## `range`

Draws a reading as rviz does: a cone along the sensor frame's x axis, with the measured range as its length and the field of view as its angle. Readings outside `[min_range, max_range]`, such as `+inf` "no detection" readings, clear it. For a fixed-distance sensor (`min_range == max_range`), a `-inf` reading is a detection, drawn at its fixed range.

| Param | Default | |
| --- | --- | --- |
| `colour` | `[255, 255, 255, 255]` | RGBA. |
| `radius` | `0.001` | Line radius, in metres. |

## `marker_array`

Logs rviz markers, each on `<entity_path>/<ns>/<id>` and posed in its header frame. For a single `visualization_msgs/Marker` topic, set `msg_type: visualization_msgs.msg.Marker`.

* Supports every marker type except `MESH_RESOURCE`.
* Supports the `ADD`, `DELETE` and `DELETEALL` actions. As in rviz, `DELETEALL` clears only the message's namespace if it has one.
* Markers expire at the end of their lifetime.
* Text markers are drawn as labels on a point, as Rerun has no 3D text.
* Markers always move with their frame, as if `frame_locked` were set.

## `occupancy_grid`

Logs maps and costmaps in the message's frame, coloured as in rviz. `/map` is latched, so give it `durability: transient_local` QoS.

| Param | Default | |
| --- | --- | --- |
| `colormap` | `RvizMap` | `RvizMap`, `RvizCostmap`, `Costmap` or `Grayscale`. |
| `opacity` | `1.0` | E.g. below 1 for a costmap drawn over the map. |
| `draw_order` | none | Grids with a higher draw order are drawn on top. |

## `odometry`

Logs the linear x and angular z velocities as plots, on `<entity_path>/vel` and `<entity_path>/ang_vel`. The robot's pose comes from TF.

## `particle_cloud`

Logs a Nav2 particle cloud as points or as arrows along each particle's heading. Needs `nav2_msgs`.

| Param | Default | |
| --- | --- | --- |
| `mode` | `points` | `points` or `arrows`. |
| `radius` | `0.0015` | Point or arrow radius, in metres. |
| `colour` | `[255, 255, 255, 255]` | Base RGBA. |
| `colour_by_weight` | `true` | Fade lighter particles. |
| `w_min`, `w_max` | from each message | Weight range for scaling. |
| `arrow_len` | `0.05` | Arrow length for the heaviest particle, in metres. |
| `arrow_axis` | `x` | Body axis giving the heading. |

## `scalars`

Plots numeric fields of any message type as time series, each on `<entity_path>/<field>` with the field's dots as slashes. The config must set the message type with `msg_type`. Fields are checked against it on start: one that does not exist, or is not a number or an array of numbers, is an error. An array is plotted as one series per element.

Values are logged as they are: non-finite ones, such as a `Range`'s `+inf` "no detection" readings, break the line. Messages with a header are logged at its stamp, others when received. A topic can be plotted alongside another module's display of it, e.g. the `range` cone.

| Param | Default | |
| --- | --- | --- |
| `fields` | required | Dotted paths to the fields to plot, e.g. `range` or `twist.twist.linear.x`. |

```yaml
- name: inlet_sensor_plot
  module: scalars
  topic: /distance_readings/inlet_transfer
  msg_type: sensor_msgs.msg.Range
  entity_path: plots/inlet_transfer
  params:
    fields: [range]
```
