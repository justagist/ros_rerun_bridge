from dataclasses import dataclass
from typing import Optional

import rerun as rr


@dataclass
class BridgeContext:
    """Shared context for all modules that run at a time."""

    urdf_tree: Optional[rr.urdf.UrdfTree] = None
    """Robot model, filled by the robot_description module once the URDF is received."""
