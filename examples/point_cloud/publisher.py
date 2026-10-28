"""Publish a rotating, rainbow-coloured helix of 2000 points at 10 Hz.

Colour is packed in an `rgb` field, as PCL does.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]


class CloudPublisher(Node):
    def __init__(self):
        super().__init__("example_point_cloud")
        self.cloud = self.create_publisher(PointCloud2, "/points", 10)
        self.t = 0.0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.t += 0.1
        s = np.linspace(0.0, 1.0, 2000)
        angle = 8 * math.pi * s + self.t
        x, y, z = np.cos(angle), np.sin(angle), 2 * s
        # Hue along the helix, packed as 0x00RRGGBB in the bytes of a float32
        r, g, b = ((np.sin(2 * math.pi * s + offset) + 1) * 127.5 for offset in (0.0, 2.1, 4.2))
        packed = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
        points = np.zeros(len(s), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("rgb", "f4")])
        points["x"], points["y"], points["z"], points["rgb"] = x, y, z, packed.view(np.float32)

        header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        self.cloud.publish(point_cloud2.create_cloud(header, FIELDS, points))


def main():
    rclpy.init()
    try:
        rclpy.spin(CloudPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
