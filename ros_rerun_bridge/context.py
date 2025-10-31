from dataclasses import dataclass
from typing import Optional

from .utils.tf_utils import TFHelper
from .utils.urdf_utils import UrdfKinematics


@dataclass
class BridgeContext:
    """Shared context for all modules that run at a time."""

    tf: TFHelper  # filled by BridgeNode
    """TF helper for logging transforms."""
    urdf_kinematics: Optional[UrdfKinematics] = None
    """URDF kinematics helper, filled by URDF module if the module is loaded correctly."""
