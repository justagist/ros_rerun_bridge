"""Publish what an RGB-D camera 1 m above the map sees of a rippling surface, at 10 Hz: a 160x120 colour image, an
aligned 16UC1 depth image in millimetres, and their camera info.

The surface is about 2 m away, with ripples moving outwards, and is painted in colour bands that follow them. As with
aligned depth on a real RGB-D camera, both images share one frame and camera info, so their pixels correspond. The
camera looks along the map's x axis, from an optical frame (x right, y down, z forward), as in ROS.
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
        self.depth = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.frame = 0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.frame += 1
        stamp = self.get_clock().now().to_msg()
        # The ripples' phase at each pixel: rings moving outwards from the image centre
        u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
        phase = np.hypot(u - WIDTH / 2, v - HEIGHT / 2) / 8.0 - self.frame / 3.0
        # Colour bands painted on the surface, so each colour stays on its ripple
        pixels = np.stack([np.sin(phase), np.sin(phase + 2.1), np.sin(phase + 4.2)], axis=-1)
        pixels = ((pixels + 1) * 127.5).astype(np.uint8)

        image = Image(height=HEIGHT, width=WIDTH, encoding="rgb8", step=3 * WIDTH, data=pixels.tobytes())
        image.header.frame_id = "camera_optical"
        image.header.stamp = stamp
        info = CameraInfo(height=HEIGHT, width=WIDTH, distortion_model="plumb_bob", d=[0.0] * 5)
        info.header = image.header
        info.k = [100.0, 0.0, WIDTH / 2, 0.0, 100.0, HEIGHT / 2, 0.0, 0.0, 1.0]
        info.p = [100.0, 0.0, WIDTH / 2, 0.0, 0.0, 100.0, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        # Ripples of +-0.25 m around 2 m
        depth_mm = (2000 + 250 * np.sin(phase)).astype(np.uint16)
        depth = Image(height=HEIGHT, width=WIDTH, encoding="16UC1", step=2 * WIDTH, data=depth_mm.tobytes())
        depth.header = image.header

        self.info.publish(info)
        self.image.publish(image)
        self.depth.publish(depth)


def main():
    rclpy.init()
    try:
        rclpy.spin(CameraPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
