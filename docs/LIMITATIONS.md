# Known limitations

What the bridge does not do yet, or does differently from rviz, and what to do instead.

## Robots and frames

* **One robot.** The bridge holds one robot model at a time: a second `robot_description` replaces the first. Several robots, e.g. in namespaces such as `/robot1`, are not supported.
* **No collision geometry.** The robot model is drawn from its visual geometry only.
* **One entity per moving transform.** Rerun only lets the transform between two frames change over time on one entity. Give `/tf` and `/tf_static` an entity each, and give [`joint_states`](MODULES.md#joint_states) the `/tf` module's entity (see [`tf`](MODULES.md#tf)). Automatic discovery sets up `/tf` and `/tf_static` this way.

## Message types

* **Types with no module**, e.g. `nav_msgs/Path`, `geometry_msgs/PolygonStamped`, `geometry_msgs/PointStamped` and interactive markers, are not shown. [`scalars`](MODULES.md#scalars) plots the numeric fields of any message. For anything else, [write a module](EXTENDING.md).
* **Images** in an encoding [`image`](MODULES.md#image) does not list, and compressed images other than JPEG and PNG, are reported and not shown. For compressed depth images, see [`compressed_depth_image`](MODULES.md#compressed_depth_image).
* **Markers** differ from rviz in a few ways: text is drawn as a label on a point, as Rerun has no 3D text; a line strip's per-point colours are not blended; meshes load without external files, such as OBJ materials and Collada textures; and markers always move with their frame. See [`marker_array`](MODULES.md#marker_array).

## Automatic discovery

See [Automatic discovery](CONFIGURATION.md#automatic-discovery) for what it shows and why.

* **Joint states do not move the robot.** They are plotted. With robot_state_publisher, the robot moves through `/tf`. Without it, the robot is drawn but stays still: configure the [`joint_states`](MODULES.md#joint_states) module.
* **Rectified images are slightly off in 3D.** A found `camera_info` topic uses the `k` intrinsics, which match raw images. Rectified images (`image_rect*`) need `p`: configure the [`camera_info`](MODULES.md#camera_info) topic with `intrinsics: p`.
* **Depth images are found by name.** Only an image topic with `depth` in its name is shown as depth. Any other depth image is shown as a plain image: configure [`depth_image`](MODULES.md#depth_image) for it.
* **Segmentation masks show as plain images.** A mask's encoding is the same as a greyscale image's. Configure [`segmentation_image`](MODULES.md#segmentation_image) to see class colours.
* **Only the root namespace's** `/tf`, `/tf_static` and `/robot_description` are found, as the bridge holds one robot.
* **Types with no built-in choice**, e.g. `nav_msgs/Odometry`, are not shown. Name a module for them in `auto.modules`, e.g. `nav_msgs/msg/Odometry: pose`.
* **Topics published with more than one message type** are not shown.
* **A new topic's first messages are missed** until the next scan finds it, up to `scan_period` (2 seconds) later. A latched topic's last message is still received.
* **A silent topic stays in view.** A topic is removed only when its publishers go. If a publisher stays but stops publishing, its last data stays in view.
* **Compressed copies of images** are recognised by image_transport's names only (`<image>/<transport>`).
* **A found topic can mix with a configured module's data** if its entity path falls beneath that module's, e.g. under a `scalars` module's `<entity_path>/<field>`. Exclude the topic.

## Remote viewing

* **No authentication or encryption.** Anyone who can reach the bridge's ports can read the stream. See [REMOTE_VIEWING.md](REMOTE_VIEWING.md).
