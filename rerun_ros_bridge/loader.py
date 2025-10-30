from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, Dict, List

import yaml
from rclpy.node import Node

from .base import ModuleSpec, TopicToComponentModule, _import_by_path
from .registry import REGISTRY


class BridgeBuilder:
    def __init__(self, node: Node) -> None:
        self.node = node
        # Ensure built-in modules are imported so their registry keys are available
        try:
            importlib.import_module("rerun_ros_bridge.modules")
        except Exception as e:
            node.get_logger().warn(f"Failed to import built-in modules: {e}")

    def build_from_config(self, cfg: Dict[str, Any]) -> List[TopicToComponentModule]:
        modules_cfg = cfg.get("modules", [])
        instances: List[TopicToComponentModule] = []
        for m in modules_cfg:
            spec = ModuleSpec(
                name=m["name"],
                module=m["module"],
                topic=m["topic"],
                entity_path=m["entity_path"],
                msg_type=m.get("msg_type"),
                qos=m.get("qos", {}),
                extra=m.get("extra", {}),
            )
            cls = self._resolve_class(spec.module)
            inst = cls(self.node, spec)
            instances.append(inst)
        return instances

    def _resolve_class(self, module_id: str):
        if REGISTRY.has(module_id):
            return REGISTRY.get(module_id)
        return _import_by_path(module_id)


def load_yaml(path: str | Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)
