from __future__ import annotations

from rclpy.time import Time

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY

try:
    from nav_msgs.msg import Odometry

    NAV_MSGS_AVAILABLE = True
except ImportError:
    NAV_MSGS_AVAILABLE = False


@REGISTRY.register("odometry")
class OdometryModule(TopicToComponentModule):
    """Module for handling Odometry messages and logging velocities to Rerun. The robot's pose comes from TF."""

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
