# Configuration

A config is a YAML file listing the topics to show and how. The bridge checks it on start, including each module's `params`: an unknown key or invalid value stops it before a viewer opens. A [module of your own](EXTENDING.md) has its `params` checked if it sets `PARAMS`.

```yaml
global_frame: map          # optional: the TF frame the 3D view is rendered in (default: map)
blueprint: layout.rbl      # optional: viewer layout to open with, relative to this file
auto: true                 # optional: also bridge the topics modules does not name; see Automatic discovery below
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

With `auto: true`, or `--auto` on the command line, the bridge also shows the topics `modules` does not name. It looks at the ROS graph every two seconds and bridges each new topic that has a publisher, if a built-in module draws its message type:

| Topics | Module |
| --- | --- |
| `/tf` | `tf`, on `tf`, drawing each frame's axes as `auto.frames` sets |
| `/tf_static` | the same, with `static: true`, on `tf_static` |
| `/robot_description` | `robot_description` |
| `Image` | `depth_image` if the topic's name contains `depth`, otherwise `image` |
| `CompressedImage` | `compressed_depth_image` for `.../compressedDepth`, otherwise `compressed_image`. Not when the raw image topic it was made from is published, as image_transport does. |
| `JointState` | `scalars`, plotting each joint's position by its name |
| `Marker`, `MarkerArray` | `marker_array` |
| `CameraInfo`, `PointCloud2`, `LaserScan`, `Range`, `PoseStamped`, `PoseArray`, `WrenchStamped`, `OccupancyGrid`, `ParticleCloud`, `Log`, `DiagnosticArray`, `Detection2DArray`, `Detection3DArray` | the module for the type |

Each topic is logged on its name as entity path, e.g. `/camera/image_raw` on `camera/image_raw`, and subscribed with QoS every publisher accepts: best effort if any publishes best effort, and transient local, for a latched topic's last message, only if all publish transient local.

The bridge leaves:

* Topics `modules` names, and topics whose entity path a module already uses, except `tf` and `tf_static`, which all frame relations share.
* Hidden topics (a name part starting with `_`), and topics with several message types.
* TF and robot descriptions outside the root namespace, e.g. `/robot1/tf`: the bridge holds one robot model.
* Joint states as robot motion: robot_state_publisher puts the joints on `/tf`, and two sources would disagree. Without it, configure the [`joint_states`](MODULES.md#joint_states) module.
* Segmentation masks, which are images too: configure [`segmentation_image`](MODULES.md#segmentation_image).
* Types no module draws alone, e.g. `Odometry`: configure them with [`scalars`](MODULES.md#scalars) or a [module of your own](EXTENDING.md).

A topic that fails to bridge, e.g. as `nav2_msgs` is not installed, is logged as a warning and left. To limit what is bridged, for instance high-bandwidth camera streams, give `auto` the topics it may bridge as [`fnmatch`](https://docs.python.org/3/library/fnmatch.html) patterns, where `*` also matches `/`. `frames` sets how TF frames are drawn, with the [`tf`](MODULES.md#tf) module's `axis_length` and `frame_names`:

```yaml
auto:
  include: ["*"]             # optional: the default
  exclude: ["/camera/*/compressed*", "/rosout"]
  frames:                    # optional
    axis_length: 0.1         # the default; null draws no axes
    frame_names: false       # the default, as many frames would crowd the view
```

With names off, the viewer still shows a frame's name: select its axes under `frames` and override *Show frame*.

`--auto` with no config bridges every topic it can, in the `map` frame unless `--global-frame` names another. With a config, `--auto` uses its `auto` filters if it has them.

## Frames

Data from stamped messages is placed in the TF frame named in the message header, as in rviz. The [`tf`](MODULES.md#tf) module links those frames into one scene; without it, data in different frames is drawn on top of each other at the origin.

`global_frame` is the frame the default 3D view is rendered in, like rviz's fixed frame: `map` for navigation, or e.g. `world` or the robot's root link for an arm. It must be linked to the data's frames, through TF or the robot's URDF, or the 3D view stays empty.

## Timeline

Data is logged on a `ros_time` timeline at each message's stamp, or at the time it was received if it has no stamp or a zero one, so the viewer can scrub back through what happened.

## Viewer layout

Without a `blueprint`, Rerun picks a layout from the data. To keep one, arrange the viewer, save it with **Save blueprint…** in the viewer's file menu, and point `blueprint` at the `.rbl` file. The layout applies even if it was saved while viewing a different recording. Layouts can also be [written in Python](https://rerun.io/docs/howto/visualization/build-a-blueprint-programmatically) and saved with `Blueprint.save()`.
