"""Run an arm and a planner node that log as a pick-and-place cycle runs, and publish the robot's diagnostics.

The arm's motor heats up while it moves, warning and then failing until it cools, and its battery drains and
recharges. The nodes log to /rosout, and the arm publishes its motor, gripper and battery statuses on /diagnostics at
2 Hz, named as a diagnostic aggregator would.
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

CYCLE = 16.0
"""Seconds in one cycle, after which it repeats."""


class Arm(Node):
    def __init__(self):
        super().__init__("example_arm")
        self.gripper = self.get_logger().get_child("gripper")
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.t = 0.0
        self.create_timer(0.5, self.tick)

    def tick(self):
        cycle = self.t % CYCLE
        self.t += 0.5
        if cycle == 0.0:
            self.get_logger().info("moving to the pick pose")
        elif cycle == 2.0:
            self.gripper.info("closing")
        elif cycle == 3.0:
            self.gripper.info("holding a part")
        elif cycle == 4.0:
            self.get_logger().info("moving to the place pose")
        elif cycle == 6.0:
            self.get_logger().warn("motor at 70 C, slowing down")
        elif cycle == 9.0:
            self.get_logger().error("motor at 85 C, stopped")
        elif cycle == 12.0:
            self.get_logger().info("motor cooled, resuming")
            self.gripper.info("opening")
        # Heats up for 10 s, then cools
        temperature = 40.0 + 5.0 * min(cycle, 10.0) - 15.0 * max(cycle - 10.0, 0.0)
        battery = 100.0 - 4.0 * cycle
        if temperature >= 85.0:
            motor = (DiagnosticStatus.ERROR, "too hot, stopped")
        elif temperature >= 70.0:
            motor = (DiagnosticStatus.WARN, "hot")
        else:
            motor = (DiagnosticStatus.OK, "ok")
        # The gripper's driver stops reporting for a while
        gripper = (DiagnosticStatus.STALE, "no data") if 13.0 <= cycle < 15.0 else (DiagnosticStatus.OK, "ok")
        charge = (DiagnosticStatus.WARN, "low") if battery < 50.0 else (DiagnosticStatus.OK, "ok")
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [
            DiagnosticStatus(name="/Robot/Arm/Motor", level=motor[0], message=motor[1]),
            DiagnosticStatus(name="/Robot/Arm/Gripper", level=gripper[0], message=gripper[1]),
            DiagnosticStatus(name="/Robot/Battery", level=charge[0], message=charge[1]),
        ]
        self.diagnostics.publish(msg)


class Planner(Node):
    def __init__(self):
        super().__init__("example_planner")
        self.t = 0.0
        self.create_timer(1.0, self.tick)

    def tick(self):
        cycle = self.t % CYCLE
        self.t += 1.0
        if cycle == 0.0:
            self.get_logger().info("planning a pick")
        elif cycle == 4.0:
            self.get_logger().info("planning a place")
        elif cycle == 9.0:
            self.get_logger().warn("arm stopped, waiting")
        elif cycle == 12.0:
            self.get_logger().info("arm resumed, replanning")


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    executor.add_node(Arm())
    executor.add_node(Planner())
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
