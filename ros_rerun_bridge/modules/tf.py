from __future__ import annotations

from rclpy.time import Time
from tf2_msgs.msg import TFMessage

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.utils import set_rr_time_from_ros


class TFParams(StrictParamsModel):
    """Parameters for the TFModule."""

    static: bool = False
    """Log transforms as static, i.e. valid at all times. Set for /tf_static."""


@REGISTRY.register("tf")
class TFModule(TopicToComponentModule):
    """Module for forwarding tf2 transforms to Rerun as relations between named frames.

    Other modules then only name the frame their data is in (`header.frame_id`), as in rviz.

    Rerun requires a frame relation to always be logged on the same entity, so use one entity for /tf and another for
    /tf_static. Log /tf_static on `tf_static`, the entity Rerun's URDF importer also logs the robot's joints on.

    Module-specific extra configuration parameters:
      - static: Log transforms as static (default: False). Set for /tf_static.
    """

    PARAMS = TFParams
    params: TFParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return TFMessage

    def handle(self, msg: TFMessage) -> None:  # noqa: D102
        # One row per transform: each has its own stamp
        for tf in msg.transforms:
            if not self.params.static:
                set_rr_time_from_ros(Time.from_msg(tf.header.stamp))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(
                self.entity_path,
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    quaternion=[q.x, q.y, q.z, q.w],
                    parent_frame=tf.header.frame_id,
                    child_frame=tf.child_frame_id,
                ),
                static=self.params.static,
            )
