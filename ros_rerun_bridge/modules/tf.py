from __future__ import annotations

from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..utils.frame_axes import FrameAxes, FrameAxesParams
from ..utils.utils import set_rr_time_from_ros, stamp_or_now


class TFParams(FrameAxesParams):
    """Parameters for the TFModule."""

    static: bool = False
    """Log transforms as static, i.e. valid at all times. Set for /tf_static."""


@REGISTRY.register("tf")
class TFModule(TopicToComponentModule):
    """Module for forwarding tf2 transforms to Rerun as relations between named frames.

    Other modules then only name the frame their data is in (`header.frame_id`), as in rviz.

    Rerun requires a frame relation's changes over time to be logged on one entity; a static value on another entity
    is allowed, and acts as a default. Use one module for /tf and another, with `static`, for /tf_static, the latter
    on `tf_static`: the entity Rerun's URDF importer logs the robot's joint defaults on.
    """

    PARAMS = TFParams
    params: TFParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and set up drawing the frames' axes.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        self._frame_axes = FrameAxes(self.params)

    @classmethod
    def ros_msg_type(cls):
        return TFMessage

    def handle(self, msg: TFMessage) -> None:
        # One row per transform: each has its own stamp
        for tf in msg.transforms:
            if not self.params.static:
                set_rr_time_from_ros(stamp_or_now(Time.from_msg(tf.header.stamp), self.node))
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
            self._frame_axes.add(tf.header.frame_id, tf.child_frame_id)
