"""Publish a 160x120 image of moving colour bands at 10 Hz, with its camera info, from a camera 1 m above the map.

The camera looks along the map's x axis. Its frame is an optical frame (x right, y down, z forward), as in ROS.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

WIDTH, HEIGHT = 160, 120


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("example_camera")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        mount = TransformStamped(child_frame_id="camera_optical")
        mount.header.frame_id = "map"
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.transform.translation.z = 1.0
        # From the map's x forward, z up to the optical frame's z forward, y down
        rotation = mount.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = -0.5, 0.5, -0.5, 0.5
        self.create_publisher(TFMessage, "/tf_static", latched).publish(TFMessage(transforms=[mount]))

        self.info = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self.image = self.create_publisher(Image, "/camera/image_raw", 10)
        self.frame = 0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.frame += 1
        stamp = self.get_clock().now().to_msg()
        # Diagonal bands that drift to the right
        u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
        phase = (u + v - 4 * self.frame) / 20.0
        pixels = np.stack([np.sin(phase), np.sin(phase + 2.1), np.sin(phase + 4.2)], axis=-1)
        pixels = ((pixels + 1) * 127.5).astype(np.uint8)

        image = Image(height=HEIGHT, width=WIDTH, encoding="rgb8", step=3 * WIDTH, data=pixels.tobytes())
        image.header.frame_id = "camera_optical"
        image.header.stamp = stamp
        info = CameraInfo(height=HEIGHT, width=WIDTH, distortion_model="plumb_bob", d=[0.0] * 5)
        info.header = image.header
        info.k = [100.0, 0.0, WIDTH / 2, 0.0, 100.0, HEIGHT / 2, 0.0, 0.0, 1.0]
        info.p = [100.0, 0.0, WIDTH / 2, 0.0, 0.0, 100.0, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info.publish(info)
        self.image.publish(image)


def main():
    rclpy.init()
    try:
        rclpy.spin(CameraPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
