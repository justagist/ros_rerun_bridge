from __future__ import annotations

from typing import Any

from nav_msgs.msg import Odometry
from rclpy.time import Time

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("odometry")
class OdometryModule(TopicToComponentModule):
    @classmethod
    def ros_msg_type(cls):
        return Odometry

    def _extract_time(self, msg) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
        # Log velocities as time-series
        rr.log(self.entity_path + "/vel", rr.Scalars(msg.twist.twist.linear.x))
        rr.log(self.entity_path + "/ang_vel", rr.Scalars(msg.twist.twist.angular.z))

        stamp = self._extract_time(msg)
        for tfp in self.extra.get("tf_paths", []):
            self.context.tf.log_tf_path(
                path=tfp["path"],
                child_frame=tfp["child_frame"],
                parent_frame=tfp["parent_frame"],
                time=stamp,
            )
