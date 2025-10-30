from __future__ import annotations

import argparse

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import rerun as rr

from .loader import BridgeBuilder, load_yaml
from .tf_utils import TFHelper


class BridgeNode(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__("rerun_ros_bridge")

        # Allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Optional TF helper accessible to modules via node.get_parameters or by storing here
        self.tf = TFHelper(self)

        # Load config
        cfg = load_yaml(config_path)

        # TODO: Log the real map once https://github.com/rerun-io/rerun/issues/1531 is merged
        rr.log(
            f"{cfg.get('global_frame', 'map')}/box",
            rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            static=True,
        )

        # build modules
        builder = BridgeBuilder(self)
        self.modules = builder.build_from_config(cfg)


def run(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="ROS2 → Rerun bridge (config + plugins)")
    parser.add_argument("--config", required=True, help="Path to YAML config")
    rr.script_add_args(parser)

    args, unknownargs = parser.parse_known_args(argv)
    rr.script_setup(args, "rerun_ros_bridge")

    rclpy.init(args=unknownargs)
    node = BridgeNode(args.config)

    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
