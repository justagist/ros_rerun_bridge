# Extending ros-rerun-bridge with a new module

Modules are small classes that subscribe to **one ROS message type** and log it to Rerun under an `entity_path`. They get access to the ROS node and a shared context (e.g. the robot model).

## Write a module

A module can live in your own package. For example, `my_pkg/my_pose.py`:

```python
from __future__ import annotations

import rerun as rr
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

from ros_rerun_bridge.base import TopicToComponentModule
from ros_rerun_bridge.types import StrictParamsModel


class MyPoseParams(StrictParamsModel):
    # Your module's typed params; unknown ones are an error
    radius: float = 0.02


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

Load it in a config by its Python path, with `my_pkg` importable by the bridge (e.g. installed in its environment, or on `PYTHONPATH`):

```yaml
modules:
  - name: pose
    module: my_pkg.my_pose:MyPoseModule
    topic: /pose
    entity_path: pose
    params:
      radius: 0.05
```

It needs no registering, unlike a built-in module. The path must name a `TopicToComponentModule` subclass, or the bridge stops with an error.

**Conventions**

* Inherit from `TopicToComponentModule`.
* Implement `ros_msg_type()` to return the message type. Skip it only if users must always set `msg_type` in the config, as for `scalars`. Users can also override it with `msg_type` (e.g. `visualization_msgs.msg.Marker`), for a module that handles several message types.
* Implement `handle(msg)`, called for every message.
* Implement `_extract_time(msg)` to log at the message's stamp. Without it, or for a zero stamp, data is logged at the time it was received.
* Set `PARAMS` to a `StrictParamsModel` subclass for the config's `params` to be validated into `self.params`.
* Errors raised while handling a message are logged, rate-limited, and the message is skipped.

**What you can use**

* `self.log(*archetypes, frame_id=...)`: log to the module's entity, placed in a named TF frame.
* `set_rr_time_from_ros(stamp)` (from `ros_rerun_bridge.utils.utils`): set Rerun's time to a ROS stamp, for messages holding several stamped items, as the `tf` module does.
* `self.context.urdf_tree`: the robot model (Rerun's `UrdfTree`), once the `robot_description` module has received it.
* `self.callback_group`: the module's callback group. Add timers to it, so they never run at the same time as `handle()`.

## Contribute a built-in module

Built-in modules live in `ros_rerun_bridge/modules/`, use relative imports (`from ..base import TopicToComponentModule`), and are registered under a name users can give as `module:`:

```python
from ..registry import REGISTRY


@REGISTRY.register("my_pose")
class MyPoseModule(TopicToComponentModule): ...
```

The name must match the file name (`my_pose` in `ros_rerun_bridge/modules/my_pose.py`): the builder imports a built-in module only when a config uses it. Document it in [MODULES.md](MODULES.md).

The test fixtures in [tests/conftest.py](../tests/conftest.py) run a module without a ROS graph and read back what it logged:

```python
from geometry_msgs.msg import PoseStamped


def test_logs_the_position(node, build, recording):
    build("my_pose", entity_path="pose")  # builds the module from a config entry
    msg = PoseStamped()
    msg.header.frame_id, msg.pose.position.x = "map", 1.0
    node.publish(msg)  # delivers it as the executor would
    row = recording.rows("/pose")[0]  # what the module logged
    assert row["Points3D:positions"] == [[1.0, 0.0, 0.0]]
    assert row["CoordinateFrame:frame"] == ["map"]
```

Run the tests with `uv run pytest`, with ROS sourced. To try a module live, drive it with `ros2 topic pub` or a bag.

## FAQ

**Should a module handle multiple topics?**
Keep modules to one responsibility. If you need several subscriptions, use separate modules, or a composite module with clearly separated params.

**Can I log to multiple Rerun entities from one module?**
Yes. Derive the paths from `self.entity_path`, or add params to control them.

**What about the module's lifecycle?**
Modules are created once at startup, and the base class creates the subscription. Do any other setup in `__init__`, after calling `super().__init__`.
