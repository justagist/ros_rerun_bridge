# Extending ros-rerun-bridge with a new module

Modules are small classes that subscribe to **one ROS message type** and log it to Rerun under an `entity_path`. They get access to the ROS node and a shared context (e.g. the robot model).

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
from ..types import StrictParamsModel

class MyPoseParams(StrictParamsModel):
    # add your own strict, typed params here
    radius: float = 0.02

@REGISTRY.register("my_pose")
class MyPoseModule(TopicToComponentModule):
    """Log a PoseStamped's position as a point."""

    PARAMS = MyPoseParams
    params: MyPoseParams  # for type checkers

    @classmethod
    def ros_msg_type(cls):
        return PoseStamped

    def _extract_time(self, msg: PoseStamped) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        # Placed in the message's TF frame, which the `tf` module links to the rest of the scene
        self.log(rr.Points3D([[p.x, p.y, p.z]], radii=self.params.radius), frame_id=msg.header.frame_id)
```

**Key conventions**

* Inherit from `TopicToComponentModule`.
* Provide `ros_msg_type()` **unless** you require users to set `msg_type` in YAML.
* If you define `PARAMS = <PydanticModel>`, the bridge will parse `params: {…}` into that model and attach it as `self.params`.
* Implement `handle(self, msg)`, which is called for every message.
* Optionally implement `_extract_time(self, msg) -> rclpy.time.Time | None` if you want Rerun timelines to align with ROS stamps.

### 3) Add it to the package (optional for local use)

If you're contributing to the package, the registry name must match the file name (`my_pose` in `ros_rerun_bridge/modules/my_pose.py`). The builder imports a built-in module only when a config uses it.

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
    entity_path: pose
    params:
      radius: 0.05
```

### 5) QoS & message types

* Users can override your module's message type with the config's `msg_type` field, by its Python path (e.g. `visualization_msgs.msg.Marker`), for a module that handles several message types.
* The bridge turns the config's `qos: { … }` into the subscription's `QoSProfile`; mention in your docs if your topic needs e.g. `transient_local`.

### 6) Testing

The test fixtures in [tests/conftest.py](../tests/conftest.py) run a module without a ROS graph and read back what it logged:

```python
def test_logs_the_position(node, build, recording):
    build("my_pose", entity_path="pose")           # builds the module from a config entry
    msg = PoseStamped()
    msg.header.frame_id, msg.pose.position.x = "map", 1.0
    node.publish(msg)                               # delivers it as the executor would
    row = recording.rows("/pose")[0]               # what the module logged
    assert row["Points3D:positions"] == [[1.0, 0.0, 0.0]]
    assert row["CoordinateFrame:frame"] == ["map"]
```

Run the tests with `uv run pytest`, with ROS sourced. To try a module live, drive it with `ros2 topic pub` or a bag.

### 7) Common utilities you can use

* `self.log(*archetypes, frame_id=...)`: log to the module's entity, placed in a named TF frame.
* `set_rr_time_from_ros(stamp)`: set Rerun's time to a ROS stamp, for messages holding several stamped items (the bridge already sets it from `_extract_time`).
* `context.urdf_tree`: the robot model (Rerun's `UrdfTree`), once the `robot_description` module has received it.

---

### FAQ

**Should a module handle multiple topics?**
Keep modules to one responsibility. If you need multiple subscriptions, consider separate modules or a composite module with clearly separated params.

**Can I log to multiple Rerun entities from one module?**
Yes. Derive the paths from `self.entity_path`, or add params to control them.

**What about lifecycle?**
Modules are created once at startup, and the base class creates the subscription. Do any other setup in `__init__` after calling `super().__init__`; timers should use `self.callback_group`, so they never run at the same time as `handle()`.
