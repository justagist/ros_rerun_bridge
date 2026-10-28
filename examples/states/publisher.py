"""Publish a robot's mode, cycling through idle, moving and charging, and a warning light that blinks while moving.

The mode is a string on /robot_mode, the light a bool on /warning_light, both at 10 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# (mode, seconds in it)
MODES = [("idle", 2.0), ("moving", 4.0), ("charging", 3.0)]


class StatesPublisher(Node):
    def __init__(self):
        super().__init__("example_states")
        self.mode = self.create_publisher(String, "/robot_mode", 10)
        self.light = self.create_publisher(Bool, "/warning_light", 10)
        self.t = 0.0
        self.create_timer(0.1, self.tick)

    def tick(self):
        self.t += 0.1
        cycle = self.t % sum(seconds for _, seconds in MODES)
        for mode, seconds in MODES:
            if cycle < seconds:
                break
            cycle -= seconds
        self.mode.publish(String(data=mode))
        # On for half of every second while moving
        self.light.publish(Bool(data=mode == "moving" and cycle % 1.0 < 0.5))


def main():
    rclpy.init()
    try:
        rclpy.spin(StatesPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
