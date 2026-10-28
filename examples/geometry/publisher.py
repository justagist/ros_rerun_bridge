"""Publish a robot pose with covariance, guesses at that pose, and a force-torque sensor's readings, at 10 Hz.

The robot drives in a 1 m circle, its position uncertainty growing and shrinking, on /robot_pose. A cloud of 30
guesses at its pose, as a localiser might spread them, is on /pose_guesses. A force-torque sensor on a tool, 1 m up at
the side, reads a force that swings round and a twisting torque, on /wrench.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

GUESSES = 30


def yawed(x: float, y: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


class GeometryPublisher(Node):
    def __init__(self):
        super().__init__("example_geometry")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tool = TransformStamped(child_frame_id="tool")
        tool.header.frame_id = "map"
        tool.header.stamp = self.get_clock().now().to_msg()
        tool.transform.translation.x, tool.transform.translation.z = 2.5, 1.0
        tool.transform.rotation.w = 1.0
        self.create_publisher(TFMessage, "/tf_static", latched).publish(TFMessage(transforms=[tool]))

        self.pose = self.create_publisher(PoseWithCovarianceStamped, "/robot_pose", 10)
        self.guesses = self.create_publisher(PoseArray, "/pose_guesses", 10)
        self.wrench = self.create_publisher(WrenchStamped, "/wrench", 10)
        # The same spread each run, scaled by the current uncertainty
        self.spread = np.random.default_rng(0).standard_normal((GUESSES, 3))
        self.t = 0.0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.t += 0.1
        stamp = self.get_clock().now().to_msg()
        heading = 0.4 * self.t
        x, y, yaw = math.cos(heading), math.sin(heading), heading + math.pi / 2
        # Standard deviations of 5 to 25 cm, longest along the direction of travel
        sigma = 0.05 + 0.1 * (1 + math.sin(0.5 * self.t))
        along, across = sigma, 0.4 * sigma
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        position_covariance = np.zeros((6, 6))
        position_covariance[:2, :2] = rotation @ np.diag([along**2, across**2]) @ rotation.T
        position_covariance[2, 2] = 0.01**2

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = "map", stamp
        msg.pose.pose = yawed(x, y, yaw)
        msg.pose.covariance = position_covariance.flatten().tolist()
        self.pose.publish(msg)

        guesses = PoseArray()
        guesses.header = msg.header
        offsets = self.spread[:, :2] * [along, across] @ rotation.T
        guesses.poses = [yawed(x + dx, y + dy, yaw + 0.2 * dyaw) for (dx, dy), dyaw in zip(offsets, self.spread[:, 2])]
        self.guesses.publish(guesses)

        wrench = WrenchStamped()
        wrench.header.frame_id, wrench.header.stamp = "tool", stamp
        wrench.wrench.force.x, wrench.wrench.force.z = 20 * math.cos(self.t), -20 + 10 * math.sin(self.t)
        wrench.wrench.torque.z = 0.6 * math.sin(2 * self.t)
        self.wrench.publish(wrench)


def main():
    rclpy.init()
    try:
        rclpy.spin(GeometryPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
