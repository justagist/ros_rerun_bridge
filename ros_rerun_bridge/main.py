"""
The main entry point for the ROS2 → Rerun bridge node.
"""

from __future__ import annotations

import argparse
import sys
from typing import TYPE_CHECKING, Any, Dict

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import rerun as rr

from . import __version__
from .base import BridgeConfig
from .context import BridgeContext
from .loader import BridgeBuilder
from .utils.tf_utils import TFHelper

if TYPE_CHECKING:
    from pathlib import Path


class BridgeNode(Node):
    """ROS2 → Rerun bridge node."""

    def __init__(self, config_path: str) -> None:
        """Create the bridge node.

        Args:
            config_path: Path to YAML config file.
        """
        super().__init__("ros_rerun_bridge")

        # Allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Optional shared context. Currently only holds TF helper.
        bridge_context = BridgeContext(tf=TFHelper(self))

        # Load config
        cfg = BridgeConfig.model_validate(load_yaml(config_path))

        # TODO: Log the real map once https://github.com/rerun-io/rerun/issues/1531 is merged
        rr.log(
            f"{cfg.global_frame}/box",
            rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            static=True,
        )

        # build modules
        builder = BridgeBuilder(self)
        self.modules = builder.build_modules(cfg, bridge_context)


def load_yaml(path: str | Path) -> Dict[str, Any]:
    """Load a YAML file from the given path."""
    with open(path, "r") as f:
        return yaml.safe_load(f)


def run(argv: list[str] | None = None) -> None:
    """Run the ROS2 → Rerun bridge node.

    Args:
        argv: Command line arguments to use. If `None`, uses `sys.argv`.
    """
    argv = sys.argv[1:] if argv is None else argv

    # Let -v/--version bypass required args
    if "--version" in argv or "-v" in argv:
        print(f"ros-rerun-bridge version {__version__}")
        sys.exit(0)

    parser = argparse.ArgumentParser(description="ROS2 → Rerun bridge (config + plugins)")

    # Group: app options
    bridge_grp = parser.add_argument_group()
    bridge_grp.add_argument("-v", "--version", action="store_true", help="Show version and exit")
    bridge_grp.add_argument("--config", required=True, help="Path to YAML config")

    # Group: Rerun SDK options (show separately)
    rerun_grp = parser.add_argument_group("Rerun options (directly passed to Rerun SDK)")
    rr.script_add_args(rerun_grp)  # <- key change: add Rerun args to this group

    # Parse known so leftover go to rclpy
    args, unknownargs = parser.parse_known_args(argv)

    rr.script_setup(args, "ros_rerun_bridge")

    rclpy.init(args=unknownargs)
    node = BridgeNode(args.config)

    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
