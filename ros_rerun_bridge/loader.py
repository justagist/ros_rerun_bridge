from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, List

from .registry import REGISTRY
from .utils.utils import _import_module_by_path

if TYPE_CHECKING:
    from pathlib import Path

    from rclpy.node import Node

    from .base import BridgeConfig, TopicToComponentModule
    from .context import BridgeContext


class BridgeBuilder:
    """Builds module instances based on a BridgeConfig."""

    def __init__(self, node: Node) -> None:
        """Initialize the builder with a ROS node.

        Args:
            node: The ROS node to use for module initialization.
        """
        self.node = node
        # Ensure built-in modules are imported so their registry keys are available
        try:
            importlib.import_module("ros_rerun_bridge.modules")
        except Exception as e:
            node.get_logger().warn(f"Failed to import built-in modules: {e}")

    def build_modules(self, cfg: BridgeConfig, context: BridgeContext) -> List[TopicToComponentModule]:
        """Build module instances based on the provided configuration.

        Args:
            cfg: The bridge configuration.
            context: Shared context for all modules.
        Returns:
            A list of instantiated TopicToComponentModule objects.
        """
        instances: List[TopicToComponentModule] = []
        for module_spec in cfg.modules:
            cls = self._resolve_class(module_spec.module)
            inst = cls(self.node, module_spec, context=context)
            instances.append(inst)
        return instances

    def _resolve_class(self, module_id: str):
        if REGISTRY.has(module_id):
            return REGISTRY.get(module_id)
        return _import_module_by_path(module_id)
