"""Publish a 10 x 8 m room with a pillar as a latched map, and a robot driving in a circle in it at 10 Hz.

The robot's pose is on /tf, its velocities on /odom, and a 180-ray laser scan, ray-cast against the map, on /scan.
The scanner is 0.2 m above the robot's base, which is on the floor.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

RESOLUTION = 0.05
WIDTH, HEIGHT = 200, 160
ORIGIN = (-5.0, -4.0)
SPEED, TURN_RATE = 0.5, 0.25
MAX_RANGE = 6.0
ANGLES = np.linspace(-math.pi, math.pi, 180, endpoint=False)


def make_grid() -> np.ndarray:
    """Occupancy (0 free, 100 occupied), indexed [row, column] from the map origin: walls and a pillar."""
    grid = np.zeros((HEIGHT, WIDTH), dtype=np.int8)
    grid[:3, :] = grid[-3:, :] = grid[:, :3] = grid[:, -3:] = 100
    grid[70:90, 140:160] = 100
    return grid


class MobileRobotPublisher(Node):
    def __init__(self):
        super().__init__("example_mobile_robot")
        self.grid = make_grid()
        grid_msg = OccupancyGrid(data=self.grid.flatten().tolist())
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = RESOLUTION
        grid_msg.info.width, grid_msg.info.height = WIDTH, HEIGHT
        grid_msg.info.origin.position.x, grid_msg.info.origin.position.y = ORIGIN
        grid_msg.info.origin.orientation.w = 1.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(OccupancyGrid, "/map", latched).publish(grid_msg)

        self.tf = self.create_publisher(TFMessage, "/tf", 10)
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.scan = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.t = 0.0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.t += 0.1
        stamp = self.get_clock().now().to_msg()
        # A circle of radius SPEED / TURN_RATE, centred left of the map's middle
        heading = TURN_RATE * self.t
        x = -1.5 + (SPEED / TURN_RATE) * math.sin(heading)
        y = (SPEED / TURN_RATE) * (1 - math.cos(heading)) - 2.0

        pose = TransformStamped(child_frame_id="base_link")
        pose.header.frame_id, pose.header.stamp = "map", stamp
        pose.transform.translation.x, pose.transform.translation.y = x, y
        pose.transform.rotation.z, pose.transform.rotation.w = math.sin(heading / 2), math.cos(heading / 2)
        scanner = TransformStamped(child_frame_id="base_scan")
        scanner.header.frame_id, scanner.header.stamp = "base_link", stamp
        scanner.transform.translation.z = 0.2
        scanner.transform.rotation.w = 1.0
        self.tf.publish(TFMessage(transforms=[pose, scanner]))

        odom = Odometry(child_frame_id="base_link")
        odom.header.frame_id, odom.header.stamp = "map", stamp
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = SPEED, TURN_RATE
        self.odom.publish(odom)

        scan = LaserScan(angle_min=float(ANGLES[0]), angle_max=float(ANGLES[-1]), range_min=0.1, range_max=MAX_RANGE)
        scan.header.frame_id, scan.header.stamp = "base_scan", stamp
        scan.angle_increment = float(ANGLES[1] - ANGLES[0])
        scan.ranges = self.ray_cast(x, y, heading).tolist()
        self.scan.publish(scan)

    def ray_cast(self, x: float, y: float, heading: float) -> np.ndarray:
        """Distance along each scan angle to the first occupied cell, or +inf beyond MAX_RANGE."""
        distances = np.arange(0.0, MAX_RANGE, RESOLUTION / 2)
        world_angles = heading + ANGLES
        columns = ((x + np.outer(distances, np.cos(world_angles)) - ORIGIN[0]) / RESOLUTION).astype(int)
        rows = ((y + np.outer(distances, np.sin(world_angles)) - ORIGIN[1]) / RESOLUTION).astype(int)
        inside = (columns >= 0) & (columns < WIDTH) & (rows >= 0) & (rows < HEIGHT)
        hit = inside & (self.grid[rows.clip(0, HEIGHT - 1), columns.clip(0, WIDTH - 1)] > 50)
        return np.where(hit.any(axis=0), distances[hit.argmax(axis=0)], math.inf).astype(np.float32)


def main():
    rclpy.init()
    try:
        rclpy.spin(MobileRobotPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
