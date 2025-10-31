from __future__ import annotations

from rclpy.time import Time

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import BaseModelWithTFPaths

try:
    from nav_msgs.msg import Odometry

    NAV_MSGS_AVAILABLE = True
except ImportError:
    NAV_MSGS_AVAILABLE = False


@REGISTRY.register("odometry")
class OdometryModule(TopicToComponentModule):
    """Module for handling Odometry messages and logging to Rerun."""

    PARAMS = BaseModelWithTFPaths
    params: BaseModelWithTFPaths  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        if not NAV_MSGS_AVAILABLE:
            raise RuntimeError("nav_msgs package is not installed.")
        return Odometry

    def _extract_time(self, msg: "Odometry") -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: "Odometry") -> None:  # noqa: D102
        # Log velocities as time-series
        rr.log(self.entity_path + "/vel", rr.Scalars(msg.twist.twist.linear.x))
        rr.log(self.entity_path + "/ang_vel", rr.Scalars(msg.twist.twist.angular.z))

        stamp = self._extract_time(msg)
        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(
                tf_path=tfp,
                time=stamp,
            )
