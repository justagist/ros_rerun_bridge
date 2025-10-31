from __future__ import annotations

from typing import Any

from std_msgs.msg import String
from urdf_parser_py import urdf as urdf_parser

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..utils.urdf_utils import UrdfKinematics


@REGISTRY.register("urdf")
class URDFModule(TopicToComponentModule):
    """Subscribe to /robot_description (std_msgs/String), parse URDF, log visuals, and expose kinematics."""

    @classmethod
    def ros_msg_type(cls):
        return String

    def handle(self, msg: Any) -> None:
        model: urdf_parser.URDF = urdf_parser.URDF.from_xml_string(msg.data)

        link_scales = dict(self.extra.get("link_scales", {}))

        kin = UrdfKinematics(model, root_path=self.entity_path, link_scales=link_scales)

        # Log once & expose kinematics to the joint states module
        kin.log_visuals()
        self.context.urdf_kinematics = kin

        # Initialize joints to zeros so transforms exist
        for j in model.joints:
            kin.set_joint(j.name, 0.0)
