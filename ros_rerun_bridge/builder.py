from __future__ import annotations

import importlib
import importlib.util
from typing import TYPE_CHECKING, List

from .registry import REGISTRY
from .utils.utils import _import_module_by_path

if TYPE_CHECKING:
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
        # Built-ins are imported only when used, so a missing optional dependency (e.g. nav2_msgs) only affects the
        # module that needs it, and its import error surfaces as is.
        builtin = f"ros_rerun_bridge.modules.{module_id}"
        if not REGISTRY.has(module_id) and module_id.isidentifier() and importlib.util.find_spec(builtin):
            importlib.import_module(builtin)
        if REGISTRY.has(module_id):
            return REGISTRY.get(module_id)
        return _import_module_by_path(module_id)
