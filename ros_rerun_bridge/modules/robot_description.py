from __future__ import annotations

from typing import Dict

from pydantic import Field, PositiveFloat
from std_msgs.msg import String
from urdf_parser_py import urdf as urdf_parser

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import BaseModelWithTFPaths
from ..utils.urdf_utils import UrdfKinematics


class RobotDescriptionParams(BaseModelWithTFPaths):
    """Parameters for the RobotDescriptionModule."""

    link_scales: Dict[str, PositiveFloat] = Field(default_factory=dict)
    """Optional per-link scale factors for visual geometry."""


@REGISTRY.register("robot_description")
class RobotDescriptionModule(TopicToComponentModule):
    """Subscribe to /robot_description (std_msgs/String), parse URDF, log visuals, and expose kinematics.

    Module-specific extra configuration parameters:
      - link_scales: Optional per-link scale factors for visual geometry.
    """

    PARAMS = RobotDescriptionParams
    params: RobotDescriptionParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return String

    def handle(self, msg: String) -> None:  # noqa: D102
        model: urdf_parser.URDF = urdf_parser.URDF.from_xml_string(msg.data)

        link_scales = dict(self.params.link_scales)

        kin = UrdfKinematics(model, root_path=self.entity_path, link_scales=link_scales)

        # Log once & expose kinematics to the joint states module
        kin.log_visuals()

        # this will allow other modules to access the kinematics (WARNING: they can modify it!)
        self.context.urdf_kinematics = kin

        # Initialize joints to zeros so transforms exist
        for j in model.joints:
            kin.set_joint(j.name, 0.0)

        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(
                tf_path=tfp,
                time=self.node.get_clock().now().to_msg(),
            )
