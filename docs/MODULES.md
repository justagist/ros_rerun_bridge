# Built-in modules

Each module subscribes to one topic and logs its messages under the module's `entity_path`. Select one with `module: <name>` in the config ([CONFIGURATION.md](CONFIGURATION.md)); set its parameters under `params`. Unknown parameters are an error.

For messages with a header, spatial modules draw the data in the header's TF frame (images in its image plane frame, see [`image`](#image)), and every module logs it at the header's stamp, or when received if the stamp is zero; see [Frames](CONFIGURATION.md#frames) and [Timeline](CONFIGURATION.md#timeline).

| Module | Message | Shows |
| --- | --- | --- |
| [`tf`](#tf) | `tf2_msgs/TFMessage` | Links between TF frames |
| [`robot_description`](#robot_description) | `std_msgs/String` | The robot model from its URDF |
| [`joint_states`](#joint_states) | `sensor_msgs/JointState` | Robot joints, without TF |
| [`image`](#image) | `sensor_msgs/Image` | Camera images |
| [`compressed_image`](#compressed_image) | `sensor_msgs/CompressedImage` | JPEG and PNG camera images |
| [`depth_image`](#depth_image) | `sensor_msgs/Image` | Depth images, also as point clouds |
| [`compressed_depth_image`](#compressed_depth_image) | `sensor_msgs/CompressedImage` | `compressedDepth` images, also as point clouds |
| [`camera_info`](#camera_info) | `sensor_msgs/CameraInfo` | Camera intrinsics, to show images in 3D |
| [`pointcloud2`](#pointcloud2) | `sensor_msgs/PointCloud2` | Point clouds |
| [`laserscan`](#laserscan) | `sensor_msgs/LaserScan` | Laser scans |
| [`range`](#range) | `sensor_msgs/Range` | Range sensor readings as cones |
| [`marker_array`](#marker_array) | `visualization_msgs/MarkerArray` or `Marker` | rviz markers |
| [`pose`](#pose) | `geometry_msgs/PoseStamped`, `PoseWithCovarianceStamped` or `nav_msgs/Odometry` | A pose as axes, with its covariance |
| [`pose_array`](#pose_array) | `geometry_msgs/PoseArray` | An arrow per pose |
| [`wrench`](#wrench) | `geometry_msgs/WrenchStamped` | Force and torque arrows |
| [`occupancy_grid`](#occupancy_grid) | `nav_msgs/OccupancyGrid` | Maps and costmaps |
| [`particle_cloud`](#particle_cloud) | `nav2_msgs/ParticleCloud` | Localisation particles |
| [`scalars`](#scalars) | Any, set with `msg_type` | Numeric fields as plots |
| [`bar_chart`](#bar_chart) | Any, set with `msg_type` | An array field as bars |
| [`states`](#states) | Any, set with `msg_type` | A field as discrete states over time |
| [`rosout`](#rosout) | `rcl_interfaces/Log` | ROS log entries |
| [`diagnostics`](#diagnostics) | `diagnostic_msgs/DiagnosticArray` | Diagnostic statuses over time |

## `tf`

Logs each transform as a link between named frames (`header.frame_id` to `child_frame_id`).

Use two `tf` modules, one for `/tf` and one for `/tf_static`, each with its own `entity_path`: Rerun only lets the transform between two frames change over time on one entity, though a static transform on another entity can act as its default. Use `entity_path: tf_static` for `/tf_static`, where the `robot_description` module also logs the robot's default joint positions.

```yaml
- name: tf
  module: tf
  topic: /tf
  entity_path: tf
- name: tf_static
  module: tf
  topic: /tf_static
  entity_path: tf_static
  qos: { depth: 10, durability: transient_local }
  params:
    static: true
```

| Param | Default | |
| --- | --- | --- |
| `static` | `false` | Log transforms as valid at all times. Set for `/tf_static`. |
| `axis_length` | none | Draw each frame's x (red), y (green) and z (blue) axes, this long in metres (above 0), as rviz's TF display does. Rerun draws none otherwise. |
| `frame_names` | `false` | Label each frame's axes with its name. Needs `axis_length`. |

Each frame's axes are drawn on an entity of their own, `frames/<frame>`, whichever module's `entity_path` the transform is on. A frame that several modules see, e.g. one from `/tf_static` and `/joint_states`, has one entity, drawn with the settings of the module that drew it last: give modules the same settings, or set them on one only. In the viewer, hide one frame's axes with the eye next to it under `frames`, or select it and override *Show frame* to show or hide its name alone.

## `robot_description`

Loads the URDF from `/robot_description` with Rerun's URDF importer and logs each link's visual geometry in the link's frame, so the robot moves with TF or with [`joint_states`](#joint_states). Give it `durability: transient_local` QoS, as the topic is latched.

* `package://` mesh paths are resolved through the sourced ROS environment.
* Collision geometry is not logged.
* Collada (`.dae`) meshes are scaled by the unit they declare (`<unit meter="...">`), as rviz does. Rerun's importer ignores it, so a mesh authored in inches would otherwise render about 40 times too large.
* Each joint's zero position is logged on `tf_static` as a default, so the robot is drawn assembled before TF or joint states arrive.
* A new model replaces the old one's geometry, e.g. when a tool is changed.
* The model is shared with other modules as `context.urdf_tree`.

## `joint_states`

Moves the robot model's joints from `/joint_states`. Needs the [`robot_description`](#robot_description) module. Joint positions are shown as reported, even outside the URDF's limits. Mimic joints follow the joint they mimic, as with `robot_state_publisher`, unless the message gives their position.

It only moves the robot model: it does not plot the joint states. To plot positions, velocities or efforts over time, use [`scalars`](#scalars) on `/joint_states`, e.g. `fields: [position]` with the joint names as `names`; to show them as bars at the time cursor, use [`bar_chart`](#bar_chart).

Not needed if `robot_state_publisher` already publishes the joints on `/tf`. Give it the `tf` module's `entity_path` (`tf`). A joint can then arrive on both `/tf` and `/joint_states` without conflict (the latest value wins, as in tf2), and `/tf` can still carry other frames, such as a camera's.

| Param | Default | |
| --- | --- | --- |
| `axis_length` | none | Draw every link's frame in the robot model as axes, fixed links included, this long in metres, as for [`tf`](#tf). |
| `frame_names` | `false` | Label each frame's axes with its name. Needs `axis_length`. |

## `image`

Logs images in the frame `<frame_id>_image_plane`, where the [`camera_info`](#camera_info) module's pinhole shows them in 3D.

Supports the RGB(A), BGR(A) and mono encodings at 8 and 16 bits, `8UC1`, `16UC1`, `32FC1`, 8-bit Bayer and `yuv422_yuy2`. Rerun displays them natively, so nothing is converted. For depth images, use [`depth_image`](#depth_image), which also draws them as point clouds.

## `compressed_image`

Logs JPEG and PNG images as they are, for the viewer to decode, in the frame `<frame_id>_image_plane` like [`image`](#image). An empty `format` is read as JPEG, as `sensor_msgs/CompressedImage` defines. For `compressedDepth` images, use [`compressed_depth_image`](#compressed_depth_image).

## `depth_image`

Logs `16UC1` and `mono16` depth images in millimetres, and `32FC1` ones in metres, as ROS's depth image conventions set them. They are logged on the image plane like [`image`](#image), so with the [`camera_info`](#camera_info) module's pinhole, Rerun also draws them as a point cloud in 3D, coloured by depth.

| Param | Default | |
| --- | --- | --- |
| `colormap` | `Turbo` | `Turbo`, `Viridis`, `Inferno`, `Magma`, `Plasma`, `Grayscale` or `CyanToYellow`. |
| `depth_range` | estimated from the data | Depths the colormap spans, in the image's units, e.g. `[500, 3000]` millimetres for `16UC1`. Rerun's estimate may be wider than the image's own range. |
| `point_fill_ratio` | Rerun's default | Size of each 3D point, relative to the gap between neighbouring points. |

## `compressed_depth_image`

Logs depth images as `compressed_depth_image_transport` publishes them (e.g. on `/camera/depth/image_raw/compressedDepth`), for the viewer to decode, like [`depth_image`](#depth_image). It takes the same params.

* `PNG`: `16UC1` only. A `32FC1` PNG holds quantised inverse depth, which Rerun would show as the wrong distances. ROS 2 Humble's format names no codec, e.g. `16UC1; compressedDepth`, and is read as PNG.
* `RVL`: `16UC1` and `32FC1`.

This uses Rerun's encoded depth images, which Rerun marks as unstable in 0.40.

## `camera_info`

Logs the camera's intrinsics as a pinhole projection from the camera frame to the image plane frame, `<frame_id>_image_plane`, where [`image`](#image) and [`compressed_image`](#compressed_image) draw the images. The intrinsics are adjusted for binning and region of interest. As in rviz, `frame_id` should be the camera's optical frame (`x` right, `y` down, `z` forward); another frame shows the image plane rotated.

| Param | Default | |
| --- | --- | --- |
| `intrinsics` | `k` | Which intrinsics match the images shown: `k` for raw images (e.g. `image_raw`), `p` (its left 3x3) for rectified ones (e.g. `image_rect`). |

## `pointcloud2`

Logs points with their colours. Points with a NaN coordinate are dropped, as is the padding at the end of an organised cloud's rows. Needs `sensor_msgs_py`.

| Param | Default | |
| --- | --- | --- |
| `colour_mode` | `auto` | `auto`: the `rgb` field if the cloud has one, otherwise no colour. `rgb_packed`: colour packed in a field named `rgb`, as PCL does. `fields`: separate channels. `none`: no colour. |
| `colour_fields` | `[r, g, b]` | Channel fields, for `colour_mode: fields`. |

## `laserscan`

Logs a scan as rays from the scanner, or as points. Needs `laser_geometry` and `sensor_msgs_py`.

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

* Supports every marker type except `MESH_RESOURCE`, and `ARROW_STRIP`, which rviz does not draw either.
* A line strip with per-point colours is drawn as a segment per point, in that point's colour: Rerun colours a whole strip, where rviz blends the colours along it.
* Supports the `ADD`, `DELETE` and `DELETEALL` actions. As in rviz, `DELETEALL` clears only the message's namespace if it has one.
* Markers expire at the end of their lifetime.
* Text markers are drawn as labels on a point, as Rerun has no 3D text.
* Markers always move with their frame, as if `frame_locked` were set.

## `pose`

Draws a pose as its x (red), y (green) and z (blue) axes, in the header's frame. For `geometry_msgs/PoseWithCovarianceStamped` or `nav_msgs/Odometry`, set `msg_type`; unstamped types such as `Pose` are an error. Their position covariance is also drawn, as an ellipsoid on `<entity_path>/covariance`, oriented along the covariance's principal axes. An all-zero orientation is treated as no rotation, as in rviz.

| Param | Default | |
| --- | --- | --- |
| `axis_length` | `0.3` | Length of the pose's axes, in metres. |
| `n_sigma` | `2` | Standard deviations the covariance ellipsoid reaches either side of the pose, along each axis. `null` to leave it out. |
| `covariance_colour` | `[255, 0, 255, 255]` | RGBA of the covariance ellipsoid. |

```yaml
- name: robot_pose
  module: pose
  topic: /amcl_pose
  msg_type: geometry_msgs.msg.PoseWithCovarianceStamped
  entity_path: robot/pose
```

## `pose_array`

Draws a `geometry_msgs/PoseArray` as an arrow per pose, along its x axis, as rviz does, e.g. waypoints or a localiser's candidate poses.

| Param | Default | |
| --- | --- | --- |
| `arrow_length` | `0.3` | Length of each arrow, in metres. |
| `radius` | `0.01` | Arrow shaft radius, in metres. |
| `colour` | `[255, 25, 25, 255]` | RGBA of the arrows. |

## `wrench`

Draws a `geometry_msgs/WrenchStamped` as a force arrow and a torque arrow from the origin of the header's frame, e.g. a force-torque sensor's readings. The torque is drawn along its axis, by the right-hand rule, rather than as rviz's curved arrow.

| Param | Default | |
| --- | --- | --- |
| `force_scale` | `0.1` | Metres of arrow per newton. |
| `torque_scale` | `0.1` | Metres of arrow per newton metre. |
| `force_colour`, `torque_colour` | `[255, 80, 80, 255]`, `[255, 220, 50, 255]` | RGBA of the arrows. |
| `radius` | `0.01` | Arrow shaft radius, in metres. |

## `occupancy_grid`

Logs maps and costmaps in the message's frame, coloured as in rviz. `/map` is latched, so give it `durability: transient_local` QoS.

| Param | Default | |
| --- | --- | --- |
| `colormap` | `RvizMap` | `RvizMap`, `RvizCostmap`, `Costmap` or `Grayscale`. |
| `opacity` | `1.0` | E.g. below 1 for a costmap drawn over the map. |
| `draw_order` | none | Grids with a higher draw order are drawn on top. |

## `particle_cloud`

Logs a Nav2 particle cloud as points or as arrows along each particle's heading. Needs `nav2_msgs`.

| Param | Default | |
| --- | --- | --- |
| `mode` | `points` | `points` or `arrows`. |
| `radius` | `0.0015` | Point or arrow radius, in metres. |
| `colour` | `[255, 255, 255, 255]` | Base RGBA. |
| `colour_by_weight` | `true` | Fade lighter particles, down to 15% of the colour's alpha. |
| `w_min`, `w_max` | from each message | Weight range for scaling. |
| `arrow_len` | `0.05` | Arrow length for the heaviest particle, in metres. |
| `arrow_axis` | `x` | Body axis giving the heading. |

## `scalars`

Plots numeric fields of any message type as time series. Set the message type with `msg_type`. Each field is plotted on `<entity_path>/<field>`, with the field's dots replaced by slashes: `twist.twist.linear.x` is plotted on `<entity_path>/twist/twist/linear/x`.

* Fields are checked against the message type on start. A field that does not exist, or is not a number or an array of numbers, is an error.
* An array is plotted as one series per element. When a message's array is shorter than earlier ones, the series for the missing elements have a gap.
* Each series follows its element's position in the array. If elements change meaning between messages, as on `/joint_states` with several publishers covering different joints, a series mixes values from different joints.
* Values are plotted as they are. Non-finite values, such as a `Range`'s `+inf` "no detection" readings, leave a gap.
* Messages are plotted at their header's stamp, or at the time they are received if they have no header or a zero stamp.
* To both draw a topic and plot it, give it two modules, e.g. `range` for the cone and `scalars` for the plot. Each subscribes to the topic on its own.

| Param | Default | |
| --- | --- | --- |
| `fields` | required | Dotted paths to the fields to plot, e.g. `range` or `twist.twist.linear.x`. |
| `style` | `lines` | `lines` or `points`. |
| `names` | entity path and element index | Series names, one per array element. |
| `colours` | Rerun's palette | RGBA per series. The last one applies to any remaining series. |
| `width` | Rerun's default | Line width, in UI points. Lines only. |
| `interpolation` | `linear` | `linear`, or `step_after`, `step_before` or `step_mid` for values that jump, such as modes or flags. Lines only. |
| `marker` | `circle` | `circle`, `diamond`, `square`, `cross`, `plus`, `up`, `down`, `left`, `right` or `asterisk`. Points only. |
| `marker_size` | Rerun's default | Marker size, in UI points. Points only. |
| `variance` | none | Dotted path to the field's variance, e.g. `variance`, drawn as a band one standard deviation either side. Must be a single number, which applies to every element of an array. Needs exactly one field. |
| `unit` | none | Unit of the values, e.g. `m`, shown in the legend. |

```yaml
- name: velocity_plot
  module: scalars
  topic: /odom
  msg_type: nav_msgs.msg.Odometry
  entity_path: plots/odom
  params:
    fields: [twist.twist.linear.x, twist.twist.angular.z]
```

With `variance` or `unit`, values are logged as Rerun measurements, drawn as lines. Many sensor messages carry a `variance`, e.g. `Range`, `Temperature` and `FluidPressure`. Rerun marks measurements as unstable in 0.40, so their recorded format may change in a later Rerun version.

```yaml
- name: temperature_plot
  module: scalars
  topic: /temperature
  msg_type: sensor_msgs.msg.Temperature
  entity_path: plots/temperature
  params:
    fields: [temperature]
    variance: variance
    unit: °C
```

The style params apply to every field of the module. To style fields differently, plot them with separate modules. The same settings can also be changed in the viewer, and kept with a [saved layout](CONFIGURATION.md#viewer-layout).

```yaml
- name: joint_positions
  module: scalars
  topic: /joint_states
  msg_type: sensor_msgs.msg.JointState
  entity_path: plots/joints
  params:
    fields: [position]
    names: [shoulder, elbow, wrist]
    style: points
    marker: diamond
```

## `bar_chart`

Draws an array field of any message type as a bar chart, one bar per element, e.g. joint efforts or battery cell voltages. Set the message type with `msg_type`. The chart shows the array at the viewer's time cursor, so scrubbing the timeline replays it.

The field is checked against the message type on start: one that does not exist, or is not an array of numbers, is an error.

The chart's y axis fits the bars shown, so it rescales as the values change: Rerun 0.40 has no setting to fix it.

| Param | Default | |
| --- | --- | --- |
| `field` | required | Dotted path to the array to draw, e.g. `effort`. |
| `colour` | Rerun's default | RGBA of the bars. |

```yaml
- name: joint_efforts
  module: bar_chart
  topic: /joint_states
  msg_type: sensor_msgs.msg.JointState
  entity_path: bars/efforts
  params:
    field: effort
```

## `states`

Shows a field of any message type as discrete states over time, e.g. a robot's mode, a controller's status, or whether a light is on. Set the message type with `msg_type`. Each change of the field's value starts a new state, drawn as a coloured lane in Rerun's state timeline view. A message older than the latest, e.g. from a bag replayed from its start, is logged whether it changed or not.

The field must be a single number, bool or string, which is checked on start, as are the config's `states` values. A value matches when it is equal at the field's precision: `1`, `1.0` and a float32 1.0 are the same state, as are `0.3` and a float32 0.3. Quote string values such as `"on"`, `"off"`, `"yes"` and `"no"`, which YAML otherwise reads as booleans.

| Param | Default | |
| --- | --- | --- |
| `field` | required | Dotted path to the field, e.g. `data` or `brightness`. |
| `states` | none | Label and colour per value: `value`, and optionally `label` and `colour` (RGBA). Give a colour for every listed state, or for none. Values not listed are still shown, labelled with the value itself, in a colour Rerun picks for it. |

```yaml
- name: robot_mode
  module: states
  topic: /robot_mode
  msg_type: std_msgs.msg.String
  entity_path: states/mode
  params:
    field: data
    states:
      - { value: idle, label: Idle, colour: [120, 120, 120, 255] }
      - { value: moving, label: Moving, colour: [40, 160, 240, 255] }
```

## `rosout`

Shows ROS log entries, from `/rosout`, in Rerun's text log view, where they can be filtered by level and searched. Each logger's entries are logged on `<entity_path>/<logger name>`, with the name's dots as slashes, so each node's logs can be shown or hidden. ROS's FATAL is shown as Rerun's CRITICAL.

Nodes publish `/rosout` with transient local durability, keeping their last 1000 entries for 10 s: subscribing the same way also gets the entries each node logged in the 10 s before the bridge started.

| Param | Default | |
| --- | --- | --- |
| `min_level` | `debug` | Least severe level shown: `debug`, `info`, `warn`, `error` or `fatal`. |

```yaml
- name: logs
  module: rosout
  topic: /rosout
  entity_path: logs
  qos: { depth: 1000, durability: transient_local }
  params:
    min_level: info
```

Each entry costs the bridge about 50 µs to receive and log, so a few hundred a second is light. `min_level` filters in the bridge, after every entry has reached it: to cut a busy node's debug output at the source, lower its log level instead, e.g. with `--ros-args --log-level info`.

## `diagnostics`

Shows `diagnostic_msgs/DiagnosticArray` statuses, from `/diagnostics` or an aggregator's `/diagnostics_agg`. Each status is logged on `<entity_path>/<status name>`, with the name split on slashes, so aggregated names such as `/Robot/Arm/Motor` nest as a tree. Its level is shown as a state, OK, WARN, ERROR or STALE, in Rerun's state timeline view, and its message in the text log view, each logged only when it changes, or when the message is older than the latest. A status's key/values are not shown: plot one with [`scalars`](#scalars) on the topic the node publishes it on, if it has one.

```yaml
- name: diagnostics
  module: diagnostics
  topic: /diagnostics
  entity_path: diagnostics
```
