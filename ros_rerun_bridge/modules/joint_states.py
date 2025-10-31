from __future__ import annotations

from sensor_msgs.msg import JointState

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("joint_states")
class JointStatesModule(TopicToComponentModule):
    """Subscribe to /joint_states (sensor_msgs/JointState) and update URDF kinematics."""

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return JointState

    def handle(self, msg: JointState) -> None:  # noqa: D102
        if not self.context.urdf_kinematics:
            self.node.get_logger().warn(
                "[joint_states] URDF kinematics not ready yet. URDF module must be loaded first."
                " Skipping joint state update."
            )
            return
        names = list(msg.name)
        # Positions are required; velocities/efforts optional
        values = list(msg.position) if msg.position is not None else []
        if not names or not values:
            return
        # Update all joints at this timestamp
        self.context.urdf_kinematics.set_joints(names, values)
