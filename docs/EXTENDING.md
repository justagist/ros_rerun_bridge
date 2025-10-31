
# Extending ros-rerun-bridge with a new module

Modules are tiny classes that subscribe to **one ROS message type** and emit **one kind of Rerun data** under an `entity_path`. They get access to the ROS node and a shared context (TF/URDF helpers).

### 1) Pick an identifier

Choose a **registry name** users can reference in YAML, e.g. `my_pose`. You can also allow loading via Python path (`pkg.mod:MyPoseModule`).

### 2) Implement the class

Create a new file in `ros_rerun_bridge/modules/my_pose.py`:

```python
from __future__ import annotations

import rerun as rr
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import BaseModelWithTFPaths

class MyPoseParams(BaseModelWithTFPaths):
    # add your own strict, typed params here
    scale: float = 1.0

@REGISTRY.register("my_pose")
class MyPoseModule(TopicToComponentModule):
    """Log PoseStamped as a 3D transform."""

    PARAMS = MyPoseParams
    params: MyPoseParams  # for type checkers

    @classmethod
    def ros_msg_type(cls):
        return PoseStamped

    def _extract_time(self, msg: PoseStamped) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        rr.log(self.entity_path, rr.Transform3D(
            translation=[p.x, p.y, p.z],
            rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            scale=[self.params.scale,]*3,
        ))

        # Optionally log TF paths configured in params
        stamp = self._extract_time(msg)
        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(tf_path=tfp, time=stamp)
```

**Key conventions**

* Inherit from `TopicToComponentModule`.
* Provide `ros_msg_type()` **unless** you require users to set `msg_type` in YAML.
* If you define `PARAMS = <PydanticModel>`, the bridge will parse `params: {…}` into that model and attach it as `self.params`.
* Implement `handle(self, msg)` – this is called for every message.
* Optionally implement `_extract_time(self, msg) -> rclpy.time.Time | None` if you want Rerun timelines to align with ROS stamps.

### 3) Add it to the package (optional for local use)

If you’re contributing to the package, import your module in `ros_rerun_bridge/modules/__init__.py` so it’s registered on import:

```python
from . import my_pose
```

For **local/private** modules, you can skip the registry and load via Python path in your YAML:

```yaml
- name: pose
  module: my_pkg.my_mod:MyPoseModule
  topic: /pose
  entity_path: map/robot
```

### 4) Use it from YAML

```yaml
modules:
  - name: pose
    module: my_pose            # or package path
    topic: /pose
    entity_path: map/robot
    params:
      scale: 1.5
      tf_paths:
        - path: map/robot
          child_frame: base_link
          parent_frame: map
```

### 5) QoS & message types

* If your module’s `ros_msg_type()` is expensive or you want to reuse the same module for multiple aliases, allow users to override using the top‑level `msg_type` field in YAML (e.g. `my_pkg_msgs/WeirdPose`).
* Expose QoS in your docs; the bridge already converts `qos: { … }` into a `QoSProfile` for your subscription.

### 6) Testing tips

* Validate your Pydantic params by constructing the model in isolation.
* Use `ros2 topic pub` or bag playback to drive the module.
* Try both `--spawn` and `--connect` flows.

### 7) Common utilities you can use

* `set_rr_time_from_ros(stamp)` – align Rerun time with a ROS stamp.
* `TFHelper.log_tf_path(tf_path, time)` – produce a transform chain under `tf_path.path` using `tf2` lookups.
* `UrdfKinematics` – if your module needs to place things in the robot frame tree.

---

### FAQ

**Should a module handle multiple topics?**
Keep modules single‑responsibility. If you need multiple subscriptions, consider separate modules or a composite module with clearly separated params.

**Can I log to multiple Rerun entities from one module?**
Yes—derive the paths from `self.entity_path` or add extra params to control sub‑paths.

**What about lifecycle?**
Modules are instantiated once at startup. Use `__init__` for setup and `self.node.create_subscription(...)` inside the base class’ constructor path is already handled for you; you only implement `handle()`.
