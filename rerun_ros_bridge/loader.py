from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any, Dict, List

import yaml

from .base import TopicToComponentModule, _import_module_by_path
from .registry import REGISTRY

if TYPE_CHECKING:
    from pathlib import Path

    from rclpy.node import Node

    from .base import BridgeConfig
    from .context import BridgeContext


class BridgeBuilder:
    def __init__(self, node: Node) -> None:
        self.node = node
        # Ensure built-in modules are imported so their registry keys are available
        try:
            importlib.import_module("rerun_ros_bridge.modules")
        except Exception as e:
            node.get_logger().warn(f"Failed to import built-in modules: {e}")

    def build_modules(self, cfg: BridgeConfig, context: BridgeContext) -> List[TopicToComponentModule]:
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


def load_yaml(path: str | Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)
