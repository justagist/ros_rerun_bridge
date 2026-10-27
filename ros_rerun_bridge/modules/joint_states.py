from __future__ import annotations

from rclpy.time import Time
from sensor_msgs.msg import JointState

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("joint_states")
class JointStatesModule(TopicToComponentModule):
    """Subscribe to /joint_states (sensor_msgs/JointState) and move the robot model's joints.

    Needs the `robot_description` module. Not needed where robot_state_publisher already puts the joints on /tf. Rerun
    requires a frame relation to always be logged on the same entity, so if the joints may also arrive on /tf, log on
    the `tf` module's entity (e.g. `tf`): the latest transform then wins, as in tf2.
    """

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return JointState

    def _extract_time(self, msg: JointState) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: JointState) -> None:  # noqa: D102
        tree = self.context.urdf_tree
        if tree is None:
            self.node.get_logger().warn(
                "[joint_states] Robot model not received yet, skipping joint states. Is robot_description configured?",
                throttle_duration_sec=5.0,
            )
            return
        for name, position in zip(msg.name, msg.position):
            joint = tree.get_joint_by_name(name)
            if joint is not None:
                # Show what the robot reports, even outside its URDF limits
                rr.log(self.entity_path, joint.compute_transform(position, clamp=False))
