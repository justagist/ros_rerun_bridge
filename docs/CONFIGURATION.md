# Configuration

A config is a YAML file listing the topics to show and how. The bridge checks it on start: an unknown key or invalid value stops it before a viewer opens. Each module's `params` are checked when the module is created, just after.

```yaml
global_frame: map          # optional: the TF frame the 3D view is rendered in (default: map)
blueprint: layout.rbl      # optional: viewer layout to open with, relative to this file
modules:
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

The [example configs](../example_configs) are a good starting point.

## Modules

`module` names a [built-in module](MODULES.md), or your own module by its Python path (`package.module:Class`, see [EXTENDING.md](EXTENDING.md)). Only the modules a config uses are loaded, so a missing optional dependency (e.g. `nav2_msgs`) only matters for the module that needs it.

`msg_type` overrides a module's message type by its Python path, e.g. `visualization_msgs.msg.Marker` to use `marker_array` on a single-marker topic.

## QoS

A subscription only receives messages from publishers with compatible QoS. Check a topic's publishers with `ros2 topic info -v <topic>`:

* **Reliability:** a `reliable` subscription (the default) receives nothing from `best_effort` publishers, which many sensor drivers use (`SensorDataQoS`). `best_effort` receives from both, so it is a safe choice for sensor topics.
* **Durability:** latched topics (`/map`, `/robot_description`, `/tf_static`, Nav2 costmaps) are published `transient_local`. Subscribe `transient_local` too, to get their last message when the bridge starts after the publisher. A `transient_local` subscription receives nothing from a `volatile` publisher.

## Frames

Data from stamped messages is placed in the TF frame named in the message header, as in rviz. The [`tf`](MODULES.md#tf) module links those frames into one scene; without it, data in different frames is drawn on top of each other at the origin.

`global_frame` is the frame the default 3D view is rendered in, like rviz's fixed frame: `map` for navigation, or e.g. `world` or the robot's root link for an arm. It must be linked to the data's frames, through TF or the robot's URDF, or the 3D view stays empty.

## Timeline

Data is logged on a `ros_time` timeline at each message's stamp, or at the time it was received if it has none, so the viewer can scrub back through what happened.

## Viewer layout

Without a `blueprint`, Rerun picks a layout from the data. To keep one, arrange the viewer, save it with **Save blueprint…** in the viewer's file menu, and point `blueprint` at the `.rbl` file. The layout applies even if it was saved while viewing a different recording. Layouts can also be [written in Python](https://rerun.io/docs/howto/visualization/build-a-blueprint-programmatically) and saved with `Blueprint.save()`.
