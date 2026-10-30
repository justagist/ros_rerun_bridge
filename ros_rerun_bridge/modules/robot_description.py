from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET

from rclpy.node import Node
from std_msgs.msg import String

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.meshes import collada_unit


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
        # Loaded in full before the old model is touched, so a URDF that fails to load leaves it in place.
        # The importer only reads files. package:// mesh paths are resolved through the ROS environment.
        with tempfile.NamedTemporaryFile("w", suffix=".urdf") as urdf_file:
            urdf_file.write(msg.data)
            urdf_file.flush()
            tree = rr.urdf.UrdfTree.from_file_path(urdf_file.name, entity_path_prefix=self.entity_path)
            links = [tree.root_link(), *(tree.get_joint_child(joint) for joint in tree.joints())]
            collision_paths = {path for link in links for path in tree.get_collision_geometry_paths(link)}
            chunks = [chunk for chunk in tree.stream().to_chunks() if chunk.entity_path not in collision_paths]
        visual_paths = {path for link in links for path in tree.get_visual_geometry_paths(link)}

        # A new model's static data only replaces the components it logs again: clear the old model's geometry. Its
        # frames are kept, as an entity without one would have no path to the scene's frames, which Rerun reports.
        for path, archetypes in self._geometry.items():
            rr.log(path, *(getattr(rr, archetype).cleared() for archetype in archetypes), static=True)
        # Logged data is batched, while chunks are sent as they are: the clears must arrive first
        rr.get_global_data_recording().flush()
        rr.send_chunks(chunks)

        self._geometry = {}
        for chunk in chunks:
            if chunk.entity_path in visual_paths:
                # Columns are named <archetype>:<component>
                names = chunk.to_record_batch().schema.names
                archetypes = {name.split(":")[0] for name in names if ":" in name} - {"CoordinateFrame"}
                self._geometry.setdefault(chunk.entity_path, set()).update(archetypes)

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
                unit = collada_unit(mesh.get("filename"))
                if unit == 1.0:
                    continue
                scale = [float(s) * unit for s in mesh.get("scale", "1 1 1").split()]
                rr.log(path, rr.InstancePoses3D.from_fields(scales=[scale]), static=True)
