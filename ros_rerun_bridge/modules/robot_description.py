from __future__ import annotations

import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import StrictParamsModel

_COLLADA_UNIT = re.compile(rb"""<unit\b[^>]*\bmeter\s*=\s*(?:"([^"]+)"|'([^']+)')""")


@REGISTRY.register("robot_description")
class RobotDescriptionModule(TopicToComponentModule):
    """Subscribe to /robot_description (std_msgs/String) and log the robot's visual geometry with Rerun's URDF importer.

    Each link's geometry is placed in the link's named frame, so the robot moves with its TF (see the `tf` module) or
    with the `joint_states` module. The joints at zero are logged as static defaults on `tf_static`. Collision geometry
    is not logged. The model is exposed in `context.urdf_tree` for other modules.
    """

    # No params, so any given are an error
    PARAMS = StrictParamsModel

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        self._geometry: dict[str, set[str]] = {}
        """The archetypes the current model logged per entity path, other than its frames."""

    @classmethod
    def ros_msg_type(cls):
        return String

    def handle(self, msg: String) -> None:
        # A new model's static data only replaces the components it logs again: clear the old model's geometry. Its
        # frames are kept, as an entity without one would have no path to the scene's frames, which Rerun reports.
        for path, archetypes in self._geometry.items():
            rr.log(path, *(getattr(rr, archetype).cleared() for archetype in archetypes), static=True)
        # Logged data is batched, while chunks are sent as they are: the clears must arrive first
        rr.get_global_data_recording().flush()
        # The importer only reads files. package:// mesh paths are resolved through the ROS environment.
        with tempfile.NamedTemporaryFile("w", suffix=".urdf") as urdf_file:
            urdf_file.write(msg.data)
            urdf_file.flush()
            tree = rr.urdf.UrdfTree.from_file_path(urdf_file.name, entity_path_prefix=self.entity_path)
            links = [tree.root_link(), *(tree.get_joint_child(joint) for joint in tree.joints())]
            collision_paths = {path for link in links for path in tree.get_collision_geometry_paths(link)}
            chunks = [chunk for chunk in tree.stream().to_chunks() if chunk.entity_path not in collision_paths]
            rr.send_chunks(chunks)

        self._geometry = {}
        for chunk in chunks:
            # Columns are named <archetype>:<component>
            archetypes = {name.split(":")[0] for name in chunk.to_record_batch().schema.names if ":" in name}
            # The joints, on their own entity, are replaced by the new model's
            if chunk.entity_path.startswith(f"/{self.entity_path.strip('/')}/") and archetypes - {"CoordinateFrame"}:
                self._geometry.setdefault(chunk.entity_path, set()).update(archetypes - {"CoordinateFrame"})

        self.context.urdf_tree = tree
        self._apply_collada_units(msg.data, tree)

    @staticmethod
    def _apply_collada_units(urdf: str, tree: rr.urdf.UrdfTree) -> None:
        """Scale Collada visual meshes by their declared `<unit meter="...">`, as rviz does.

        Rerun 0.40's Collada importer ignores the unit, so e.g. a mesh authored in inches renders ~40x too large. The
        scale is logged as the mesh's instance scale, combined with the URDF `<mesh scale>` the importer applied.

        Args:
            urdf: URDF XML, to map each visual mesh to its file and URDF scale.
            tree: The loaded robot model, for the visual geometry entity paths.
        """
        for link in ET.fromstring(urdf).iter("link"):
            # The importer lists a link's visuals in document order
            paths = tree.get_visual_geometry_paths(link.get("name"))
            for path, visual in zip(paths, link.findall("visual")):
                mesh = visual.find("geometry/mesh")
                if mesh is None or not mesh.get("filename", "").lower().endswith(".dae"):
                    continue
                unit = _collada_unit(mesh.get("filename"))
                if unit == 1.0:
                    continue
                scale = [float(s) * unit for s in mesh.get("scale", "1 1 1").split()]
                rr.log(path, rr.InstancePoses3D.from_fields(scales=[scale]), static=True)


def _collada_unit(filename: str) -> float:
    """Return the metres per unit a Collada file declares, or 1.0 if it declares none or cannot be found.

    Args:
        filename: Mesh path from the URDF: `package://`, `file://` or absolute.
    """
    try:
        if filename.startswith("package://"):
            package, _, relative = filename[len("package://") :].partition("/")
            path = Path(get_package_share_directory(package)) / relative
        else:
            path = Path(filename.removeprefix("file://"))
    except PackageNotFoundError:
        # The importer reports the missing mesh
        return 1.0
    if not path.is_file():
        return 1.0
    # <unit> is in the <asset> block at the top of the file
    with path.open("rb") as f:
        match = _COLLADA_UNIT.search(f.read(16384))
    return float(match.group(1) or match.group(2)) if match else 1.0
