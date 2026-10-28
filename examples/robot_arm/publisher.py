"""Publish robot.urdf on a latched /robot_description, and joint states at 20 Hz that swing the arm and open and close
its gripper.

Only the left finger is published: the right one mimics it in the URDF.
"""

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

URDF = Path(__file__).with_name("robot.urdf")


class ArmPublisher(Node):
    def __init__(self):
        super().__init__("example_robot_arm")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(String, "/robot_description", latched).publish(String(data=URDF.read_text()))
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.t = 0.0
        self.create_timer(0.05, self.tick)

    def tick(self):
        self.t += 0.05
        msg = JointState(name=["turntable", "elbow", "finger_left_joint"])
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [1.5 * math.sin(0.5 * self.t), 0.8 * math.sin(self.t), 0.015 * (1 + math.sin(2 * self.t))]
        self.joints.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(ArmPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
