"""Publish one marker that changes type every 2 s, and a fixed marker beside it, on /viewer_check/markers."""

import itertools

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

# (type, colour) in the order shown; each type is a different Rerun archetype
CYCLE = [
    ("CUBE", (1.0, 0.2, 0.2)),
    ("SPHERE", (0.2, 1.0, 0.2)),
    ("CYLINDER", (0.2, 0.4, 1.0)),
    ("ARROW", (1.0, 1.0, 0.2)),
    ("LINE_STRIP", (1.0, 0.2, 1.0)),
    ("TRIANGLE_LIST", (0.2, 1.0, 1.0)),
    ("TEXT_VIEW_FACING", (1.0, 1.0, 1.0)),
]


class MarkerCycler(Node):
    def __init__(self):
        super().__init__("viewer_check_markers")
        self.pub = self.create_publisher(MarkerArray, "/viewer_check/markers", 10)
        self.cycle = itertools.cycle(CYCLE)
        self.create_timer(2.0, self.tick)
        self.tick()

    def marker(self, ns, type_name, colour, x):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.action = ns, 0, Marker.ADD
        m.type = getattr(Marker, type_name)
        m.pose.position.x = x
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b = colour
        m.color.a = 1.0
        if type_name == "ARROW":
            m.scale.x, m.scale.y, m.scale.z = 1.0, 0.1, 0.1
        elif type_name == "LINE_STRIP":
            m.scale.x = 0.05
            m.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.5, y=0.5, z=0.0), Point(x=0.0, y=0.5, z=0.5)]
        elif type_name == "TRIANGLE_LIST":
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.5, y=0.0, z=0.0), Point(x=0.0, y=0.5, z=0.0)]
        elif type_name == "TEXT_VIEW_FACING":
            m.text = "TEXT"
        return m

    def tick(self):
        type_name, colour = next(self.cycle)
        self.get_logger().info(f"cycling marker is now {type_name}")
        self.pub.publish(
            MarkerArray(
                markers=[
                    self.marker("cycling", type_name, colour, 0.0),
                    self.marker("fixed", "CUBE", (0.6, 0.6, 0.6), 2.0),
                ]
            )
        )


def main():
    rclpy.init()
    try:
        rclpy.spin(MarkerCycler())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
