"""
Automatic discovery: bridges the topics a config does not name, each with the built-in module for its message type.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from .base import ModuleConfig, QoSConfig
from .utils.utils import _import_module_by_path, set_rr_time_from_ros

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.topic_endpoint_info import TopicEndpointInfo

    from .base import AutoConfig, BridgeConfig, TopicToComponentModule
    from .builder import BridgeBuilder
    from .context import BridgeContext

MODULES_BY_TYPE: dict[str, dict[str, Any]] = {
    "sensor_msgs/msg/CameraInfo": {"module": "camera_info"},
    "sensor_msgs/msg/PointCloud2": {"module": "pointcloud2"},
    "sensor_msgs/msg/LaserScan": {"module": "laserscan"},
    "sensor_msgs/msg/Range": {"module": "range"},
    "visualization_msgs/msg/MarkerArray": {"module": "marker_array"},
    "visualization_msgs/msg/Marker": {"module": "marker_array", "msg_type": "visualization_msgs.msg.Marker"},
    "geometry_msgs/msg/PoseStamped": {"module": "pose"},
    "geometry_msgs/msg/PoseArray": {"module": "pose_array"},
    "geometry_msgs/msg/WrenchStamped": {"module": "wrench"},
    "nav_msgs/msg/OccupancyGrid": {"module": "occupancy_grid"},
    "nav2_msgs/msg/ParticleCloud": {"module": "particle_cloud"},
    "rcl_interfaces/msg/Log": {"module": "rosout"},
    "diagnostic_msgs/msg/DiagnosticArray": {"module": "diagnostics"},
    "vision_msgs/msg/Detection2DArray": {"module": "detections_2d"},
    "vision_msgs/msg/Detection3DArray": {"module": "detections_3d"},
}
"""Modules for the message types that only one module draws, as config entry fields."""

SHARED_ENTITY_PATHS = {"tf", "tf_static"}
"""Entity paths a discovered module may share with a configured one: frame relations from /tf and /tf_static belong on
them, whichever module logs them, e.g. a configured `joint_states` on `tf`."""


def suggest(
    topic: str, msg_type: str, graph: dict[str, list[str]], auto: AutoConfig, tf_entity: str = "tf"
) -> list[dict[str, Any]]:
    """Pick how to bridge a topic, from its message type and name.

    Args:
        topic: The topic's full name.
        msg_type: Its message type, e.g. `sensor_msgs/msg/Image`.
        graph: The types of the bridged topics with publishers, configured or discovered, by name.
        auto: The discovery settings: the modules mapped to message types, and how to draw TF frames.
        tf_entity: The entity path /tf's transforms are logged on, for joint states to move the robot model on.

    Returns:
        The topic's config entries, without QoS: one per module to show it with, or none if no module draws it.
    """
    if any(part.startswith("_") for part in topic.split("/")):
        # Hidden topics, by ROS convention
        return []
    entry = {"name": topic, "topic": topic, "entity_path": topic.strip("/")}
    # The rules run from the most specific to the most general, and the first that applies decides: the root TF and
    # robot model topics, then the user's mappings, then the built-in choices by type, then those that also need the
    # topic's name or the other topics shown
    # Only the root namespace's: Rerun's URDF importer logs the robot's joint defaults on `tf_static`, and the bridge
    # holds one robot model, so several robots' models and frames would mix. Not remapped by `auto.modules`, as the
    # scene's frames and robot model depend on them.
    if msg_type == "tf2_msgs/msg/TFMessage" and topic == "/tf":
        return [{**entry, "module": "tf", "params": auto.frames.model_dump()}]
    if msg_type == "tf2_msgs/msg/TFMessage" and topic == "/tf_static":
        return [{**entry, "module": "tf", "params": {"static": True, **auto.frames.model_dump()}}]
    if msg_type == "std_msgs/msg/String" and topic == "/robot_description":
        return [{**entry, "module": "robot_description"}]
    if (mapped := auto.modules.get(msg_type)) is not None:
        return [{**entry, "module": mapped.module, "msg_type": msg_type.replace("/", "."), "params": mapped.params}]
    if msg_type == "sensor_msgs/msg/JointState":
        # Moved on /tf's entity: Rerun takes a frame relation's changes over time on one entity only, so a joint that
        # robot_state_publisher also puts on /tf is then one relation, whose latest value wins, as in tf2
        return [
            {
                **entry,
                "module": "scalars",
                "msg_type": "sensor_msgs.msg.JointState",
                "params": {"fields": ["position", "velocity", "effort"], "names_field": "name"},
            },
            {**entry, "module": "joint_states", "entity_path": tf_entity, "params": auto.frames.model_dump()},
        ]
    # Types only one module draws, whatever the topic's name
    if msg_type in MODULES_BY_TYPE:
        return [{**entry, **MODULES_BY_TYPE[msg_type]}]
    if msg_type == "sensor_msgs/msg/Image":
        # Only the name tells depth from other single-channel images. A depth image shown as an image still shows.
        # Segmentation masks are images too, and are left to a config.
        return [{**entry, "module": "depth_image" if "depth" in topic.lower() else "image"}]
    if msg_type == "sensor_msgs/msg/CompressedImage":
        base, _, transport = topic.rpartition("/")
        if "sensor_msgs/msg/Image" in graph.get(base, []):
            # image_transport's copy of an image already bridged raw
            return []
        return [{**entry, "module": "compressed_depth_image" if transport == "compressedDepth" else "compressed_image"}]
    return []


def qos_from_publishers(publishers: list[TopicEndpointInfo]) -> QoSConfig:
    """A subscription QoS compatible with every publisher, as rosbag2 picks one.

    Args:
        publishers: The topic's publishers.

    Returns:
        Best effort if any publisher is, else reliable. Transient local only if every publisher is, to get a latched
        topic's last message without missing the volatile publishers.
    """
    reliable = all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in publishers)
    latched = all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in publishers)
    return QoSConfig(
        reliability="reliable" if reliable else "best_effort",
        durability="transient_local" if latched else "volatile",
    )


@dataclass
class _Bridged:
    """A discovered topic's modules, and what they were built for."""

    modules: list[TopicToComponentModule]
    msg_type: str
    qos: QoSConfig
    missing_since: Time | None = None
    """When the topic was first seen without publishers, on ROS time, while it has none."""
    missing_scans: int = 0
    """Scans in a row the topic has had no publishers."""


class TopicDiscovery:
    """Bridges each topic the config does not name as it appears, if a built-in module draws its type, and drops it once
    its publishers have gone."""

    def __init__(self, node: Node, cfg: BridgeConfig, builder: BridgeBuilder, context: BridgeContext) -> None:
        """Look at the ROS graph every `auto.scan_period` seconds.

        Args:
            node: The bridge's ROS node.
            cfg: The config, with `auto` set. Its modules' topics and entity paths are left to them.
            builder: Builds the modules for discovered topics.
            context: Shared context for all modules.
        """
        self._node = node
        self._auto: AutoConfig = cfg.auto
        self._builder = builder
        self._context = context
        self._configured = {node.resolve_topic_name(m.topic) for m in cfg.modules}
        self._configured_paths = {m.entity_path.strip("/") for m in cfg.modules}
        self._tf_entity = next(
            (m.entity_path.strip("/") for m in cfg.modules if node.resolve_topic_name(m.topic) == "/tf"), "tf"
        )
        """The entity path /tf's transforms are logged on: a configured module's, or the one discovery gives it."""
        self._bridged: dict[str, _Bridged] = {}
        """Discovered topics' modules, by topic."""
        self._rejected: set[str] = set()
        """Discovered topics that failed to bridge, left until they lose their publishers, then tried again."""
        self._check_modules(builder)
        # The bridge holds one robot model: a configured one, e.g. a namespaced robot's, keeps it
        self._has_robot_model = any(m.module == "robot_description" for m in cfg.modules)
        # Steady time: on ROS time, with use_sim_time, discovery would stop while a bag is paused or not playing
        node.create_timer(
            self._auto.scan_period,
            self.scan,
            callback_group=MutuallyExclusiveCallbackGroup(),
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _check_modules(self, builder: BridgeBuilder) -> None:
        """Check that the modules mapped to message types exist and their params are valid, so a config error stops the
        bridge before a viewer opens rather than when a topic of the type appears. Checks of params against the message
        type, e.g. `scalars`'s fields, are left to when a topic appears.

        Args:
            builder: Resolves the modules.

        Raises:
            ValueError: If a module cannot be found, or its params are invalid, named in the message.
            ImportError: If a module or a message type cannot be imported, or does not exist, named in the message.
        """
        for msg_type, mapped in self._auto.modules.items():
            try:
                _import_module_by_path(msg_type.replace("/", "."))
                cls = builder._resolve_class(mapped.module)
                if cls.PARAMS is not None:
                    cls.PARAMS.model_validate(mapped.params)
            # AttributeError: a module of the path has no class of the name, e.g. a misspelled message type
            except (ValueError, ImportError, AttributeError) as e:
                error = ValueError if isinstance(e, ValueError) else ImportError
                raise error(f"[auto.modules: {msg_type}] {e}") from e

    @property
    def modules(self) -> list[TopicToComponentModule]:
        """Modules of the discovered topics bridged now."""
        return [module for bridged in self._bridged.values() for module in bridged.modules]

    def scan(self) -> None:
        """Update bridged topics whose publishers have gone or changed, then bridge each new topic that has publishers,
        passes the filters and has a module for its type. Last, drop topics that another topic now shows."""
        # Topics with publishers only: a subscriber alone, e.g. a node waiting for a topic, leaves nothing to show
        graph = {
            topic: types
            for topic, types in self._node.get_topic_names_and_types()
            if self._node.count_publishers(topic) > 0
        }
        # Configured topics are kept even if the filters exclude them, so that a compressed image is left for its raw
        # image when the config shows that
        eligible = {
            topic: types for topic, types in graph.items() if topic in self._configured or self._auto.matches(topic)
        }
        # A rejected topic whose publishers have all gone is forgotten, so it is tried again when they return
        self._rejected &= graph.keys()

        # 1. Topics already bridged: drop the stale, rebuild those whose type changed, resubscribe on new QoS
        for topic in list(self._bridged):
            self._update(topic, eligible)

        # 2. New topics: pick their modules, check their entity paths are free, build them with QoS their publishers
        # accept. A topic with several types is left alone, as no one module could subscribe to all of them.
        for topic, types in eligible.items():
            if topic in self._configured or topic in self._bridged or topic in self._rejected or len(types) != 1:
                continue
            entries = [
                entry
                for entry in suggest(topic, types[0], self._shown(eligible), self._auto, self._tf_entity)
                if not (entry["module"] == "robot_description" and self._has_robot_model)
            ]
            if not entries:
                continue
            publishers = self._node.get_publishers_info_by_topic(topic)
            if not publishers:
                # Gone since the graph was read: with none to match QoS to, look again on a later scan
                continue
            # Two modules on one entity path would mix their data, except on the TF entities, where frame relations
            # from several sources belong together. A configured /tf's own entity is shared with joint states only: any
            # other topic there could clear its frame relations.
            in_use = self._configured_paths | {module.entity_path for module in self.modules}
            if taken := [
                entry["entity_path"]
                for entry in entries
                if entry["entity_path"] in in_use - SHARED_ENTITY_PATHS
                and not (entry["module"] == "joint_states" and entry["entity_path"] == self._tf_entity)
            ]:
                self._rejected.add(topic)
                self._node.get_logger().warn(f"[auto] not bridging {topic}: entity path '{taken[0]}' is already in use")
                continue
            qos = qos_from_publishers(publishers)
            # All of the topic's modules or none: those built before one fails are closed, so no half-shown topic is
            # left subscribed
            modules = []
            try:
                for entry in entries:
                    modules.append(self._builder.build_module(ModuleConfig(**entry, qos=qos), self._context))
            except Exception as e:
                for module in modules:
                    module.close()
                self._rejected.add(topic)
                self._node.get_logger().warn(f"[auto] not bridging {topic}: {type(e).__name__}: {e}")
                continue
            self._bridged[topic] = _Bridged(modules, types[0], qos)
        # 3. Topics no longer shown: their choice depends on what else is shown, which step 2 may have changed
        for topic, bridged in list(self._bridged.items()):
            # e.g. a compressed image, once its raw image is bridged too
            if topic in eligible and not suggest(
                topic, bridged.msg_type, self._shown(eligible), self._auto, self._tf_entity
            ):
                self._drop(topic, "another topic shows it", clear_at=self._node.get_clock().now())

    def _shown(self, eligible: dict[str, list[str]]) -> dict[str, list[str]]:
        """The types of the topics shown now, configured or bridged, so a compressed image is left for its raw image
        only if that is shown.

        Args:
            eligible: The types of the topics with publishers that may be bridged, by name.

        Returns:
            The types of those of them shown, by name.
        """
        return {
            topic: types for topic, types in eligible.items() if topic in self._configured or topic in self._bridged
        }

    def _update(self, topic: str, eligible: dict[str, list[str]]) -> None:
        """Drop a bridged topic once it has had no publishers for `auto.stale_scans` scans, unless it never goes stale.
        Drop it to be rebuilt this scan if its type changed, and subscribe it again if its publishers' QoS changed.

        Args:
            topic: The bridged topic.
            eligible: The types of the topics with publishers that may be bridged, by name.
        """
        bridged = self._bridged[topic]
        types = eligible.get(topic)
        # No publishers: count the scans before dropping it, so a restarting node keeps its topic
        if types is None:
            # A latched topic's last message stays true, and its module keeps what it needs to update it, e.g. a robot
            # model's links to replace
            if bridged.qos.durability == "transient_local" or self._auto.is_never_stale(topic):
                return
            if bridged.missing_since is None:
                bridged.missing_since = self._node.get_clock().now()
            bridged.missing_scans += 1
            if bridged.missing_scans >= self._auto.stale_scans:
                self._drop(topic, "it has no publishers", clear_at=bridged.missing_since)
            return
        bridged.missing_since, bridged.missing_scans = None, 0
        publishers = self._node.get_publishers_info_by_topic(topic)
        if not publishers:
            # Gone since the graph was read: the next scan counts it as missing
            return
        if types != [bridged.msg_type]:
            # What the old type drew no longer holds
            self._drop(topic, f"its type is now {', '.join(types)}", clear_at=self._node.get_clock().now())
            return
        # A new publisher can need other QoS, e.g. a best-effort one joining reliable ones, which a reliable
        # subscription would not hear
        qos = qos_from_publishers(publishers)
        if qos != bridged.qos:
            for module in bridged.modules:
                module._resubscribe(qos)
            bridged.qos = qos
            self._node.get_logger().info(f"[auto] subscribed to {topic} again: its publishers' QoS changed")

    def _drop(self, topic: str, reason: str, clear_at: Time | None = None) -> None:
        """Close a bridged topic's modules, and clear what they drew.

        Args:
            topic: The bridged topic.
            reason: Why, for the log.
            clear_at: When to clear what the modules drew, on ROS time, so the viewer shows nothing stale from then on.
                Not cleared if None.
        """
        bridged = self._bridged.pop(topic)
        for module in bridged.modules:
            module.close()
        # Cleared at a point on the timeline, so earlier data stays for scrubbing back. Each module decides what
        # clearing means, e.g. TF and plots keep theirs.
        if clear_at is not None:
            set_rr_time_from_ros(clear_at)
            for module in bridged.modules:
                module.clear()
        self._node.get_logger().info(f"[auto] dropped {topic}: {reason}")
