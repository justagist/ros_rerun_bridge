# Configuration

A config is a YAML file listing the topics to show and how. The bridge checks it on start, including each module's `params`: an unknown key or invalid value stops it before a viewer opens. A [module of your own](EXTENDING.md) has its `params` checked if it sets `PARAMS`.

```yaml
global_frame: map          # optional: the TF frame the 3D view is rendered in (default: map)
blueprint: layout.rbl      # optional: viewer layout to open with, relative to this file
auto: true                 # optional: also show topics not listed here; see Automatic discovery
modules:                   # optional with auto
  - name: scan             # required: a name for logs
    module: laserscan      # required: a built-in module, or a Python path to your own
    topic: /scan           # required: the topic to subscribe to
    entity_path: scan      # required: where to log in Rerun
    msg_type: pkg.msg.Type # optional: message type, for modules that accept several
    qos:                   # optional: subscription QoS, every field optional
      depth: 10
      reliability: best_effort   # or reliable (the default); see QoS below
      durability: volatile       # or transient_local, for latched topics
      history: keep_last         # or keep_all
    params:                # optional: the module's parameters
      mode: lines
```

The [examples](../examples) are a good starting point.

## Modules

`module` names a [built-in module](MODULES.md), or your own module by its Python path (`package.module:Class`, see [EXTENDING.md](EXTENDING.md)). Only the modules a config uses are loaded, so a missing optional dependency (e.g. `nav2_msgs`) only matters for the module that needs it.

`msg_type` overrides a module's message type by its Python path, e.g. `visualization_msgs.msg.Marker` to use `marker_array` on a single-marker topic.

## QoS

A subscription only receives messages from publishers with compatible QoS. Check a topic's publishers with `ros2 topic info -v <topic>`:

* **Reliability:** a `reliable` subscription (the default) receives nothing from `best_effort` publishers, which many sensor drivers use (`SensorDataQoS`). `best_effort` receives from both, so it is a safe choice for sensor topics.
* **Durability:** latched topics (`/map`, `/robot_description`, `/tf_static`, Nav2 costmaps) are published `transient_local`. Subscribe `transient_local` too, to get their last message when the bridge starts after the publisher. A `transient_local` subscription receives nothing from a `volatile` publisher.

## Automatic discovery

Instead of listing every topic, you can let the bridge find them. Turn it on with `--auto` on the command line, or `auto: true` in the config:

```bash
ros-rerun-bridge --auto --global-frame world
```

No config is needed. With a config, the bridge shows its topics as configured, and finds the rest.

### What is shown

Every 2 seconds by default, the bridge looks for new topics. It shows a topic if the topic has a publisher and a built-in module can draw its message type:

| Message type | Shown with |
| --- | --- |
| `TFMessage` on `/tf` and `/tf_static` | `tf`, with each frame drawn as axes (see [Settings](#settings)) |
| `String` on `/robot_description` | `robot_description` |
| `Image` | `depth_image` if the topic name contains `depth`, otherwise `image` |
| `CompressedImage` | `compressed_depth_image` for topics ending in `compressedDepth`, otherwise `compressed_image` |
| `JointState` | `scalars`, plotting each joint's position |
| `Marker`, `MarkerArray` | `marker_array` |
| `CameraInfo` | `camera_info` |
| `PointCloud2` | `pointcloud2` |
| `LaserScan` | `laserscan` |
| `Range` | `range` |
| `PoseStamped` | `pose` |
| `PoseArray` | `pose_array` |
| `WrenchStamped` | `wrench` |
| `OccupancyGrid` | `occupancy_grid` |
| `ParticleCloud` | `particle_cloud` |
| `Log` | `rosout` |
| `DiagnosticArray` | `diagnostics` |
| `Detection2DArray` | `detections_2d` |
| `Detection3DArray` | `detections_3d` |

Each topic is logged under its own name: `/camera/image_raw` becomes `camera/image_raw`. The bridge picks QoS that works with every publisher of the topic, so best-effort sensor topics and latched topics such as `/map` need no settings.

### What is not shown

* **Topics the config lists.** They keep their config. A topic whose entity path a configured module already uses is skipped too, except `tf` and `tf_static`, which are meant to be shared.
* **Compressed copies of a raw image.** image_transport publishes both. If the raw image is shown, its compressed copy is not. A compressed copy shown before its raw image appeared is removed once the raw image is shown.
* **Joint states as robot motion.** robot_state_publisher already moves the robot through `/tf`, and two sources would disagree. Joint states are plotted instead. Without robot_state_publisher, configure the [`joint_states`](MODULES.md#joint_states) module.
* **A second robot.** The bridge holds one robot model, so `/robot_description` is skipped if the config has a `robot_description` module, and only the root `/tf`, `/tf_static` and `/robot_description` are found, not e.g. `/robot1/tf`.
* **Hidden topics** (a part of the name starts with `_`), and topics published with more than one message type.
* **Types no module draws on its own**, e.g. `Odometry`. Configure them with [`scalars`](MODULES.md#scalars) or a [module of your own](EXTENDING.md).

Segmentation masks are images too, so they show as plain images. Configure [`segmentation_image`](MODULES.md#segmentation_image) to see their class colours.

If a configured module logs beneath its entity path, e.g. `scalars` on `<entity_path>/<field>`, a found topic can land on one of those paths and mix with it. Exclude that topic (see [Settings](#settings)).

### When topics come and go

* **A topic is removed** when 3 scans in a row find no publisher for it: 4 to 6 seconds with the default settings. The wait lets a restarting node keep its topic. A topic whose publisher is still running is never removed, however rarely it publishes. To keep a topic even when its publishers go, list it in `never_stale` (see [Settings](#settings)).
* **Latched topics are never removed**, e.g. a map or the robot model: their last message stays true after the publisher goes.
* **Stale data is cleared** when a topic is removed, so a stopped camera's last image does not look current. Earlier data stays on the timeline. TF, plots and logs are not cleared, as they stay true or show history. Only the topic's own data is cleared, not another topic's beneath it, e.g. `/scan/filtered` under `/scan`.
* **A removed topic comes back** when a publisher appears again.
* **A topic whose message type changes** is shown again with the module for its new type. What the old type drew is cleared.
* **A topic is subscribed again** when a new publisher needs different QoS, e.g. a volatile publisher joining a latched topic. What it shows stays as it is.
* **A topic that cannot be shown**, e.g. because `nav2_msgs` is not installed, logs one warning. The bridge tries it again once its publishers go and come back.

### Settings

`auto` also takes settings, all optional:

```yaml
auto:
  include: ["*"]                                  # topics that may be shown (the default: all)
  exclude: ["/camera/*/compressed*", "/rosout"]   # topics never shown
  frames:                                         # how TF frames are drawn
    axis_length: 0.1                              # the default; null draws no axes
    frame_names: false                            # the default
  scan_period: 2.0                                # seconds between looks for topics (the default)
  stale_scans: 3                                  # scans without a publisher before a topic is removed (the default)
  never_stale: ["/camera/*"]                      # topics never removed or cleared (the default: none)
```

* `include` and `exclude` take [`fnmatch`](https://docs.python.org/3/library/fnmatch.html) patterns. `*` also matches `/`, so `/camera/*` covers every topic under `/camera`. Use `exclude` for high-bandwidth topics you do not need, such as extra camera streams.
* `scan_period` and `stale_scans` set how quickly topics are found and removed. A topic is removed at most `scan_period` x `stale_scans` seconds after its last publisher goes.
* `never_stale` takes patterns too. A matching topic stays subscribed, and its data stays in view, while it has no publisher. Use it for a topic you want to keep seeing, such as a sensor that is only started now and then.
* `frames` takes the [`tf`](MODULES.md#tf) module's `axis_length` and `frame_names`. Names are off by default, as many frames crowd the view. To show one frame's name, select its axes under `frames` in the viewer and override *Show frame*.

`--auto` without a config uses these defaults, and the `map` frame unless `--global-frame` names another. With a config, `--auto` uses the config's `auto` settings, if it has any.

## Frames

Data from stamped messages is placed in the TF frame named in the message header, as in rviz. The [`tf`](MODULES.md#tf) module links those frames into one scene; without it, data in different frames is drawn on top of each other at the origin.

`global_frame` is the frame the default 3D view is rendered in, like rviz's fixed frame: `map` for navigation, or e.g. `world` or the robot's root link for an arm. It must be linked to the data's frames, through TF or the robot's URDF, or the 3D view stays empty.

## Timeline

Data is logged on a `ros_time` timeline at each message's stamp, or at the time it was received if it has no stamp or a zero one, so the viewer can scrub back through what happened.

## Viewer layout

Without a `blueprint`, Rerun picks a layout from the data. To keep one, arrange the viewer, save it with **Save blueprint…** in the viewer's file menu, and point `blueprint` at the `.rbl` file. The layout applies even if it was saved while viewing a different recording. Layouts can also be [written in Python](https://rerun.io/docs/howto/visualization/build-a-blueprint-programmatically) and saved with `Blueprint.save()`.
