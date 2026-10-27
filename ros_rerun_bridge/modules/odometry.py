from __future__ import annotations

from nav_msgs.msg import Odometry
from rclpy.time import Time

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("odometry")
class OdometryModule(TopicToComponentModule):
    """Module for handling Odometry messages and logging velocities to Rerun. The robot's pose comes from TF."""

    @classmethod
    def ros_msg_type(cls):
        return Odometry

    def _extract_time(self, msg: Odometry) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Odometry) -> None:
        # Log velocities as time-series
        rr.log(self.entity_path + "/vel", rr.Scalars(msg.twist.twist.linear.x))
        rr.log(self.entity_path + "/ang_vel", rr.Scalars(msg.twist.twist.angular.z))
