# rerun-ros-bridge

A modular and config-driven ROS2 → Rerun bridge. Define topic mappings in YAML and
compose behavior via small plugin modules (one topic ↦ one rerun component).

## Install (dev)
```bash
pip install -e .

# Ensure your ROS2 distro is installed & sourced (e.g., Humble):
#   source /opt/ros/humble/setup.bash
```

## Run
```bash
rerun-ros-bridge --config example.config.yaml --spawn-viewer
```
Any remaining CLI args after `--` are forwarded to `rclpy`.

## Config schema
```yaml
modules:
  - name: rgb
    module: image              # registry key or python path to class
    topic: /camera/color/image_raw
    entity_path: map/robot/camera/img
    # optional: override ROS msg class
    # msg_type: sensor_msgs.msg.Image
    qos: { depth: 10, reliability: reliable }
    extra: { encoding_override: null }

  - name: rgb_camera_info
    module: camera_info
    topic: /camera/color/camera_info
    entity_path: map/robot/camera

  - name: depth_points
    module: pointcloud2
    topic: /camera/depth/points
    entity_path: map/robot/camera/points
    extra:
      color_mode: rgb_packed   # rgb_packed | fields | none
      # color_fields: [r, g, b]

  - name: scan
    module: laserscan
    topic: /scan
    entity_path: map/robot/scan
    extra: { mode: lines, origin_scale: 0.3, radius: 0.0025 }

  - name: odom
    module: odometry
    topic: /odom
    entity_path: odometry

  - name: urdf
    module: urdf
    topic: /robot_description
    entity_path: map/robot/urdf
    qos: { depth: 1, durability: transient_local }
    extra: { fix_camera_link_scale: 0.00254 }
```

### Notes
- `module` can be a registry key (e.g., `image`) or a full Python path
  like `rerun_ros_bridge.modules.image:ImageModule`.
- `msg_type` is optional; modules default to their canonical ROS msg class.
  You can override this (e.g., for custom message aliases).
- `extra` is module-specific config.
- Basic QoS knobs are exposed (`depth`, `reliability`, `durability`, `history`).

## Extending
To add a new module:
```python
from rerun_ros_bridge.base import TopicToComponentModule
from rerun_ros_bridge.registry import REGISTRY
import rerun as rr

@REGISTRY.register("my_pose")
class PoseModule(TopicToComponentModule):
    @classmethod
    def ros_msg_type(cls):
        from geometry_msgs.msg import PoseStamped
        return PoseStamped

    def _extract_time(self, msg):
        from rclpy.time import Time
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg):
        t = msg.pose.position
        q = msg.pose.orientation
        rr.log(self.entity_path, rr.Transform3D(translation=[t.x,t.y,t.z], rotation=rr.Quaternion(xyzw=[q.x,q.y,q.z,q.w])))
```
Then update your YAML:
```yaml
- name: pose
  module: my_pose
  topic: /pose
  entity_path: map/robot
```

## TF usage
A `TFHelper` is available on the node (`node.tf`) for modules needing tf lookups.
You can inject frame ids via `extra` and call `node.tf.log_tf_path(…)` inside your module.
