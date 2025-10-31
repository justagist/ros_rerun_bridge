from dataclasses import dataclass
from typing import Optional

from .utils.tf_utils import TFHelper
from .utils.urdf_utils import UrdfKinematics


@dataclass
class BridgeContext:
    tf: TFHelper  # filled by BridgeNode
    urdf_kinematics: Optional[UrdfKinematics] = None  # filled by URDF module once model is loaded
