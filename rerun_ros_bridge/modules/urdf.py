from __future__ import annotations

from typing import Any

import trimesh
from std_msgs.msg import String

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..utils.urdf_utils import load_urdf_from_msg, log_scene


@REGISTRY.register("urdf")
class URDFModule(TopicToComponentModule):
    @classmethod
    def ros_msg_type(cls):
        return String

    def handle(self, msg: Any) -> None:
        urdf = load_urdf_from_msg(msg)

        # Optional fixup for camera scale (example carried over; configurable)
        fix_camera = self.extra.get("fix_camera_link_scale", 0.0)
        if fix_camera:
            scene = urdf.scene
            orig, _ = scene.graph.get("camera_link")

            scene.graph.update(frame_to="camera_link", matrix=orig.dot(trimesh.transformations.scale_matrix(fix_camera)))
            scaled = scene.scaled(1.0)
        else:
            scaled = urdf.scene

        base_link = urdf.base_link or self.extra.get("base_link", "base_link")
        log_scene(scene=scaled, node=base_link, path=self.entity_path, static=True)
