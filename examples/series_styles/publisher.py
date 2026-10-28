"""Publish three sine-wave joint positions at 10 Hz, and a mode (0, 1, 2) that changes every 2 s."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


class SeriesPublisher(Node):
    def __init__(self):
        super().__init__("viewer_check_series")
        self.joints = self.create_publisher(JointState, "/viewer_check/joints", 10)
        self.mode = self.create_publisher(Int32, "/viewer_check/mode", 10)
        self.t = 0.0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.t += 0.1
        msg = JointState(name=["shoulder", "elbow", "wrist"])
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [math.sin(self.t + phase) for phase in (0.0, 2.0, 4.0)]
        self.joints.publish(msg)
        self.mode.publish(Int32(data=int(self.t / 2) % 3))


def main():
    rclpy.init()
    try:
        rclpy.spin(SeriesPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
