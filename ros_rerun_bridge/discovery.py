"""
Automatic discovery: bridges the topics a config does not name, each with the built-in module for its message type.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from .base import ModuleConfig, QoSConfig

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.topic_endpoint_info import TopicEndpointInfo

    from .base import AutoConfig, BridgeConfig, TopicToComponentModule
    from .builder import BridgeBuilder
    from .context import BridgeContext

SCAN_PERIOD = 2.0
"""Seconds between looks at the ROS graph for new topics."""

MODULES_BY_TYPE: dict[str, dict[str, Any]] = {
    "sensor_msgs/msg/CameraInfo": {"module": "camera_info"},
    "sensor_msgs/msg/PointCloud2": {"module": "pointcloud2"},
    "sensor_msgs/msg/LaserScan": {"module": "laserscan"},
    "sensor_msgs/msg/Range": {"module": "range"},
    # Plotted only: moving the robot model from them could disagree with /tf, which robot_state_publisher fills
    "sensor_msgs/msg/JointState": {
        "module": "scalars",
        "msg_type": "sensor_msgs.msg.JointState",
        "params": {"fields": ["position"], "names_field": "name"},
    },
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


def suggest(topic: str, msg_type: str, graph: dict[str, list[str]]) -> dict[str, Any] | None:
    """Pick how to bridge a topic, from its message type and name.

    Args:
        topic: The topic's full name.
        msg_type: Its message type, e.g. `sensor_msgs/msg/Image`.
        graph: The types of every topic with publishers, by name.

    Returns:
        The topic's config entry, without QoS, or None if no module draws it.
    """
    if any(part.startswith("_") for part in topic.split("/")):
        # Hidden topics, by ROS convention
        return None
    entry = {"name": topic, "topic": topic, "entity_path": topic.strip("/")}
    if msg_type in MODULES_BY_TYPE:
        return {**entry, **MODULES_BY_TYPE[msg_type]}
    # Only the root namespace's: Rerun's URDF importer logs the robot's joint defaults on `tf_static`, and the bridge
    # holds one robot model, so several robots' models and frames would mix
    if msg_type == "tf2_msgs/msg/TFMessage" and topic == "/tf":
        return {**entry, "module": "tf"}
    if msg_type == "tf2_msgs/msg/TFMessage" and topic == "/tf_static":
        return {**entry, "module": "tf", "params": {"static": True}}
    if msg_type == "std_msgs/msg/String" and topic == "/robot_description":
        return {**entry, "module": "robot_description"}
    if msg_type == "sensor_msgs/msg/Image":
        # Only the name tells depth from other single-channel images. A depth image shown as an image still shows.
        # Segmentation masks are images too, and are left to a config.
        return {**entry, "module": "depth_image" if "depth" in topic.lower() else "image"}
    if msg_type == "sensor_msgs/msg/CompressedImage":
        base, _, transport = topic.rpartition("/")
        if "sensor_msgs/msg/Image" in graph.get(base, []):
            # image_transport's copy of an image already bridged raw
            return None
        return {**entry, "module": "compressed_depth_image" if transport == "compressedDepth" else "compressed_image"}
    return None


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


class TopicDiscovery:
    """Bridges each topic the config does not name as it appears, if a built-in module draws its type."""

    def __init__(self, node: Node, cfg: BridgeConfig, builder: BridgeBuilder, context: BridgeContext) -> None:
        """Look at the ROS graph every `SCAN_PERIOD` seconds.

        Args:
            node: The bridge's ROS node.
            cfg: The config, with `auto` set. Its modules' topics and entity paths are left to them.
            builder: Builds the modules for discovered topics.
            context: Shared context for all modules.
        """
        self._node = node
        self._filters: AutoConfig = cfg.auto
        self._builder = builder
        self._context = context
        self.modules: list[TopicToComponentModule] = []
        """Modules built for discovered topics."""
        self._handled = {node.resolve_topic_name(m.topic) for m in cfg.modules}
        """Topics bridged, by the config or by discovery, or that discovery failed to bridge."""
        self._entity_paths = {m.entity_path.strip("/") for m in cfg.modules}
        node.create_timer(SCAN_PERIOD, self.scan, callback_group=MutuallyExclusiveCallbackGroup())

    def scan(self) -> None:
        """Bridge each new topic that has publishers, passes the filters and has a module for its type."""
        graph = {
            topic: types
            for topic, types in self._node.get_topic_names_and_types()
            if self._node.count_publishers(topic) > 0
        }
        for topic, types in graph.items():
            if topic in self._handled or len(types) != 1 or not self._filters.matches(topic):
                continue
            entry = suggest(topic, types[0], graph)
            if entry is None:
                continue
            self._handled.add(topic)
            if entry["entity_path"] in self._entity_paths - SHARED_ENTITY_PATHS:
                self._node.get_logger().warn(
                    f"[auto] not bridging {topic}: entity path '{entry['entity_path']}' is already in use"
                )
                continue
            try:
                qos = qos_from_publishers(self._node.get_publishers_info_by_topic(topic))
                module = self._builder.build_module(ModuleConfig(**entry, qos=qos), self._context)
            # Any error: one would stop the executor, and with it the bridge, over a single topic
            except Exception as e:
                self._node.get_logger().warn(f"[auto] not bridging {topic}: {type(e).__name__}: {e}")
                continue
            self._entity_paths.add(entry["entity_path"])
            self.modules.append(module)
