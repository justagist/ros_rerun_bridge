from __future__ import annotations

import time

from pydantic import Field
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState

import rerun as rr

from ..base import JointField, JointSource, ModuleConfig, TopicToComponentModule
from ..context import HANDOVER, BridgeContext
from ..registry import REGISTRY
from ..utils.frame_axes import FrameAxes, FrameAxesParams


class JointStatesParams(FrameAxesParams):
    """Parameters for the JointStatesModule."""

    prefer: JointSource = "tf"
    """Which source moves a joint that also arrives on /tf, e.g. from robot_state_publisher: `tf`, the fewest rows to
    log; `joint_states`, e.g. as they come more often; or `both`, the latest value winning, the most load. A preferred
    source that stops sending a joint hands it to the other after a second."""
    plot: list[JointField] = Field(default_factory=list)
    """The fields to plot over time, each joint as a series named after it, on `<plot_entity_path>/<field>/<joint>`."""
    plot_entity_path: str = "joint_states"
    """The entity the plots are logged beneath, as `entity_path` is the `tf` module's, for the joints."""


@REGISTRY.register("joint_states")
class JointStatesModule(TopicToComponentModule):
    """Subscribe to /joint_states (sensor_msgs/JointState), move the robot model's joints and plot their fields.

    Moving the joints needs the `robot_description` module. Rerun requires a frame relation's changes over time to be
    logged on one entity, so if the joints may also arrive on /tf, log on the `tf` module's entity (e.g. `tf`).
    """

    PARAMS = JointStatesParams
    params: JointStatesParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and set up drawing the robot's link frames.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        self._frame_axes = FrameAxes(self.params)
        self._tree: rr.urdf.UrdfTree | None = None
        """The robot model the joints' frames were read from, so a new model gets its frames read and drawn too."""
        self._child_frames: dict[str, str] = {}
        """Each joint's child frame, as Rerun and /tf name it, by joint name."""
        self._series: set[str] = set()
        """The plotted series' entity paths, whose names are logged once."""
        self._warned_no_model = False
        """Whether the missing robot model was reported: once, as with automatic discovery there may never be one."""

    @classmethod
    def ros_msg_type(cls):
        return JointState

    def _extract_time(self, msg: JointState) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: JointState) -> None:
        tree = self.context.urdf_tree
        if tree is not None:
            self._move(msg, tree)
        elif not self._warned_no_model:
            self.node.get_logger().warn(
                "[joint_states] Robot model not received yet, not moving joints until it is. Is "
                "robot_description configured?"
            )
            self._warned_no_model = True
        # After moving the robot, so that a field that cannot be plotted does not stop it
        self._plot(msg)

    def _move(self, msg: JointState, tree: rr.urdf.UrdfTree) -> None:
        """Log the joints' transforms, moving the robot model.

        Args:
            msg: The joint states.
            tree: The robot model.
        """
        if tree is not self._tree:
            self._child_frames = {}
            for joint in tree.joints():
                transform = joint.compute_transform(0.0, clamp=False)
                # The frame names as Rerun has them, e.g. with the URDF's frame prefix
                parent, child = (
                    frame.as_arrow_array().to_pylist()[0] for frame in (transform.parent_frame, transform.child_frame)
                )
                self._child_frames[joint.name] = child
                if self.params.axis_length is not None:
                    # Every link, fixed ones included: messages only carry the joints that move
                    self._frame_axes.add(parent, child)
            self._tree = tree
        positions = dict(zip(msg.name, msg.position))
        # Mimic joints follow the joint they mimic, as robot_state_publisher does, unless the message gives them.
        # Repeat until no more can be derived, for joints mimicking a mimic joint.
        pending = [joint for joint in tree.joints() if joint.mimic is not None and joint.name not in positions]
        while derived := [joint for joint in pending if joint.mimic.joint in positions]:
            for joint in derived:
                positions[joint.name] = joint.mimic.multiplier * positions[joint.mimic.joint] + joint.mimic.offset
            pending = [joint for joint in pending if joint not in derived]
        now = time.monotonic()
        for name, position in positions.items():
            joint = tree.get_joint_by_name(name)
            if joint is None:
                continue
            frame = self._child_frames[name]
            if self.params.prefer == "tf":
                received = self.context.tf_frame_times.get(frame)
                if received is not None and now - received < HANDOVER:
                    continue
            elif self.params.prefer == "joint_states":
                self.context.joint_state_frame_times[frame] = now
            # Show what the robot reports, even outside its URDF limits
            rr.log(self.entity_path, joint.compute_transform(position, clamp=False))

    def _plot(self, msg: JointState) -> None:
        """Log the fields in `plot`, each joint as a series of its own named after it.

        Args:
            msg: The joint states.

        Raises:
            ValueError: If a plotted field has values, but not one per joint name.
        """
        for field in self.params.plot:
            values = getattr(msg, field)
            # e.g. velocity and effort, left empty by publishers that do not measure them
            if len(values) == 0:
                continue
            if len(values) != len(msg.name):
                raise ValueError(f"'{field}' has {len(values)} values for the {len(msg.name)} joint names")
            for name, value in zip(msg.name, values):
                path = f"{self.params.plot_entity_path}/{field}/{rr.escape_entity_path_part(name)}"
                if path not in self._series:
                    rr.log(path, rr.SeriesLines(names=[name]), static=True)
                    self._series.add(path)
                rr.log(path, rr.Scalars([value]))

    def clear(self) -> None:
        """Keep the joints, on the `tf` module's entity that other frame relations share, and the plots, as history."""
