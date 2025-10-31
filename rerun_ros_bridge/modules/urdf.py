from __future__ import annotations

from typing import Any, Dict

from pydantic import Field, PositiveFloat
from std_msgs.msg import String
from urdf_parser_py import urdf as urdf_parser

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.urdf_utils import UrdfKinematics


class URDFParams(StrictParamsModel):
    link_scales: Dict[str, PositiveFloat] = Field(default_factory=dict)


@REGISTRY.register("urdf")
class URDFModule(TopicToComponentModule):
    """Subscribe to /robot_description (std_msgs/String), parse URDF, log visuals, and expose kinematics."""

    PARAMS = URDFParams
    params: URDFParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return String

    def handle(self, msg: Any) -> None:
        model: urdf_parser.URDF = urdf_parser.URDF.from_xml_string(msg.data)

        link_scales = dict(self.params.link_scales)

        kin = UrdfKinematics(model, root_path=self.entity_path, link_scales=link_scales)

        # Log once & expose kinematics to the joint states module
        kin.log_visuals()
        self.context.urdf_kinematics = kin

        # Initialize joints to zeros so transforms exist
        for j in model.joints:
            kin.set_joint(j.name, 0.0)
