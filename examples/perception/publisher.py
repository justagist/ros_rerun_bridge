"""Publish what a camera 1 m above the map sees of a ball and a box moving over a floor, at 10 Hz, and what a perception
model makes of it: a 320x240 mono8 image and its camera info, a mono8 segmentation mask, and 2D and 3D detections.

The ball circles the image centre and the box slides from side to side, passing behind the ball. The model labels 8x8
pixel cells, so its mask's edges are blocky, and it misses the ball's shadowed side. Its 2D detections are boxes
around each class's labelled cells, with a score and a tracking id, and its 3D detections place them 1 m from the
camera. Everything shares the image's frame, stamp and camera info, so their pixels correspond. The camera looks along
the map's x axis, from an optical frame (x right, y down, z forward), as in ROS.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

WIDTH, HEIGHT = 320, 240
FOCAL_LENGTH = 200.0
FLOOR, BALL, BOX = 0, 1, 2
TRACK_IDS = {BALL: "17", BOX: "23"}
CELL = 8
# How far the 3D detections are from the camera, in metres
DEPTH = 1.0
# The floor's texture, fixed as the camera does not move
TEXTURE = np.random.default_rng(0).normal(0, 6, (HEIGHT, WIDTH))


class PerceptionPublisher(Node):
    def __init__(self):
        super().__init__("example_perception")
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
        self.segmentation = self.create_publisher(Image, "/camera/segmentation", 10)
        self.detections_2d = self.create_publisher(Detection2DArray, "/camera/detections", 10)
        self.detections_3d = self.create_publisher(Detection3DArray, "/camera/detections_3d", 10)
        self.frame = 0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.frame += 1
        t = self.frame / 10.0
        u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
        ball_u, ball_v = WIDTH / 2 + 100 * np.cos(t), HEIGHT / 2 + 60 * np.sin(t)
        ball = np.hypot(u - ball_u, v - ball_v) < 35
        box_u = WIDTH / 2 + 110 * np.sin(0.6 * t)
        box = (np.abs(u - box_u) < 40) & (np.abs(v - (HEIGHT / 2 + 30)) < 40)
        # A textured floor darkening towards the top, a box with grain along it, and a ball lit from the top left
        floor = 40 + 50 * v / HEIGHT + TEXTURE
        grain = 150 + 15 * np.sin((u - box_u) / 3)
        ball_shade = 230 - 2.5 * np.hypot(u - ball_u + 12, v - ball_v + 12)
        # The ball is listed first, so it is in front where the two overlap
        pixels = np.select([ball, box], [ball_shade, grain], floor).clip(0, 255).astype(np.uint8)
        # The model's labels: one per cell, from its centre pixel, with the ball's dim side taken for what is behind it
        labels = np.select([ball & (pixels > 140), box], [BALL, BOX], FLOOR).astype(np.uint8)
        classes = labels[CELL // 2 :: CELL, CELL // 2 :: CELL].repeat(CELL, axis=0).repeat(CELL, axis=1)

        image = Image(height=HEIGHT, width=WIDTH, encoding="mono8", step=WIDTH, data=pixels.tobytes())
        image.header.frame_id = "camera_optical"
        image.header.stamp = self.get_clock().now().to_msg()
        info = CameraInfo(height=HEIGHT, width=WIDTH, distortion_model="plumb_bob", d=[0.0] * 5)
        info.header = image.header
        info.k = [FOCAL_LENGTH, 0.0, WIDTH / 2, 0.0, FOCAL_LENGTH, HEIGHT / 2, 0.0, 0.0, 1.0]
        info.p = [FOCAL_LENGTH, 0.0, WIDTH / 2, 0.0, 0.0, FOCAL_LENGTH, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        segmentation = Image(height=HEIGHT, width=WIDTH, encoding="mono8", step=WIDTH, data=classes.tobytes())
        segmentation.header = image.header
        detections_2d = Detection2DArray(header=image.header)
        detections_3d = Detection3DArray(header=image.header)
        for class_id, score in ((BALL, 0.9 + 0.05 * np.sin(t)), (BOX, 0.75 + 0.1 * np.cos(0.6 * t))):
            rows, columns = np.nonzero(classes == class_id)
            if not len(rows):
                continue
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id, result.hypothesis.score = str(class_id), float(score)
            detection = Detection2D(results=[result], id=TRACK_IDS[class_id])
            bbox = detection.bbox
            bbox.size_x, bbox.size_y = float(columns.max() + 1 - columns.min()), float(rows.max() + 1 - rows.min())
            bbox.center.position.x = columns.min() + bbox.size_x / 2
            bbox.center.position.y = rows.min() + bbox.size_y / 2
            detections_2d.detections.append(detection)
            # Back through the pinhole to DEPTH, as deep as it is wide
            scale = DEPTH / FOCAL_LENGTH
            detection_3d = Detection3D(results=[result], id=detection.id)
            centre = detection_3d.bbox.center
            centre.position.x = (bbox.center.position.x - WIDTH / 2) * scale
            centre.position.y = (bbox.center.position.y - HEIGHT / 2) * scale
            centre.position.z = DEPTH
            centre.orientation.w = 1.0
            size = detection_3d.bbox.size
            size.x, size.y, size.z = bbox.size_x * scale, bbox.size_y * scale, bbox.size_x * scale
            detections_3d.detections.append(detection_3d)

        self.info.publish(info)
        self.image.publish(image)
        self.segmentation.publish(segmentation)
        self.detections_2d.publish(detections_2d)
        self.detections_3d.publish(detections_3d)


def main():
    rclpy.init()
    try:
        rclpy.spin(PerceptionPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
