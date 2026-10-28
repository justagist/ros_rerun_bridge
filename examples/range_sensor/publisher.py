"""Publish a range sensor 0.5 m above the map, tilted down, reading a distance that rises and falls at 10 Hz.

For 1 s in every 5 it detects nothing, and reads +inf.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Range
from tf2_msgs.msg import TFMessage


class RangePublisher(Node):
    def __init__(self):
        super().__init__("example_range_sensor")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        mount = TransformStamped(child_frame_id="range_sensor")
        mount.header.frame_id = "map"
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.transform.translation.z = 0.5
        # Pitched 30 degrees down about y
        mount.transform.rotation.y, mount.transform.rotation.w = math.sin(math.pi / 12), math.cos(math.pi / 12)
        self.create_publisher(TFMessage, "/tf_static", latched).publish(TFMessage(transforms=[mount]))

        self.range = self.create_publisher(Range, "/range", 10)
        self.t = 0.0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.t += 0.1
        msg = Range(radiation_type=Range.ULTRASOUND, field_of_view=0.4, min_range=0.05, max_range=2.0)
        msg.header.frame_id = "range_sensor"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.range = math.inf if self.t % 5 > 4 else 1.0 + 0.6 * math.sin(self.t)
        self.range.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(RangePublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
