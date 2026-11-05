"""Automatic discovery picks a module per topic, subscribes with QoS its publishers accept, and leaves configured
topics and entity paths alone."""

import pytest
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from ros_rerun_bridge.base import BridgeConfig
from ros_rerun_bridge.builder import BridgeBuilder
from ros_rerun_bridge.discovery import MODULES_BY_TYPE, TopicDiscovery, suggest
from ros_rerun_bridge.modules.compressed_image import CompressedImageModule
from ros_rerun_bridge.modules.laserscan import LaserScanModule

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def discover(node, context):
    """Start discovery for a config, e.g. discover(auto={"exclude": ["/rosout"]}, modules=[...])."""

    def _discover(**config) -> TopicDiscovery:
        cfg = BridgeConfig(**{"auto": True, **config})
        builder = BridgeBuilder(node)
        builder.build_modules(cfg, context)
        return TopicDiscovery(node, cfg, builder, context)

    return _discover


def subscribed(node) -> dict[str, object]:
    """Subscriptions by topic."""
    return {subscription.topic: subscription for subscription in node.subscriptions}


@pytest.mark.parametrize(
    ("topic", "msg_type", "module"),
    [
        ("/scan", "sensor_msgs/msg/LaserScan", "laserscan"),
        ("/tf", "tf2_msgs/msg/TFMessage", "tf"),
        ("/robot_description", "std_msgs/msg/String", "robot_description"),
        ("/camera/image_raw", "sensor_msgs/msg/Image", "image"),
        ("/camera/depth/image_rect_raw", "sensor_msgs/msg/Image", "depth_image"),
        ("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage", "compressed_image"),
        ("/camera/depth/image_raw/compressedDepth", "sensor_msgs/msg/CompressedImage", "compressed_depth_image"),
    ],
)
def test_picks_the_module_from_the_type_and_name(topic, msg_type, module):
    assert suggest(topic, msg_type, {topic: [msg_type]})["module"] == module


def test_tf_static_is_logged_as_static():
    entry = suggest("/tf_static", "tf2_msgs/msg/TFMessage", {})
    assert (entry["module"], entry["params"], entry["entity_path"]) == ("tf", {"static": True}, "tf_static")


def test_joint_states_are_plotted_by_joint_name():
    # Not moving the robot model: /tf from robot_state_publisher does that, and both would disagree
    entry = suggest("/joint_states", "sensor_msgs/msg/JointState", {})
    assert entry["module"] == "scalars"
    assert entry["params"] == {"fields": ["position"], "names_field": "name"}


@pytest.mark.parametrize(
    ("topic", "msg_type"),
    [
        ("/robot1/tf", "tf2_msgs/msg/TFMessage"),
        ("/robot1/robot_description", "std_msgs/msg/String"),
        ("/chatter", "std_msgs/msg/String"),
        ("/odom", "nav_msgs/msg/Odometry"),
        ("/_hidden/scan", "sensor_msgs/msg/LaserScan"),
        ("/scan/_raw", "sensor_msgs/msg/LaserScan"),
    ],
)
def test_leaves_topics_no_module_draws_or_that_are_hidden_or_namespaced_robots(topic, msg_type):
    assert suggest(topic, msg_type, {topic: [msg_type]}) is None


def test_leaves_image_transport_copies_of_a_raw_image():
    graph = {"/camera/image_raw": ["sensor_msgs/msg/Image"]}
    assert suggest("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage", graph) is None
    assert suggest("/camera/image_raw/compressedDepth", "sensor_msgs/msg/CompressedImage", graph) is None


def test_bridges_topics_as_they_appear(node, discover):
    discovery = discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    node.advertise("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage")
    discovery.scan()
    discovery.scan()
    assert [type(m) for m in discovery.modules] == [LaserScanModule, CompressedImageModule]
    assert [s.topic for s in node.subscriptions] == ["/scan", "/camera/image_raw/compressed"]
    assert discovery.modules[0].entity_path == "scan"


def test_scans_on_a_timer(node, discover):
    discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    node.timers[-1]()
    assert "/scan" in subscribed(node)


def test_bridged_messages_are_logged(node, discover, recording):
    discovery = discover()
    node.advertise("/joint_states", "sensor_msgs/msg/JointState")
    discovery.scan()
    node.publish(JointState(name=["elbow"], position=[0.5]))
    rows = recording.rows("/joint_states/position/elbow")
    assert [row["Scalars:scalars"] for row in rows if not row["static"]] == [[0.5]]


def test_marker_topics_are_drawn_as_markers(node, discover):
    discovery = discover()
    node.advertise("/marker", "visualization_msgs/msg/Marker")
    discovery.scan()
    assert subscribed(node)["/marker"].msg_type is Marker


def test_leaves_topics_without_publishers(node, discover):
    discovery = discover()
    # Only subscribed to, e.g. a command topic: no publisher to receive from or to match the QoS of
    node.topics["/scan"] = ["sensor_msgs/msg/LaserScan"]
    discovery.scan()
    assert node.subscriptions == []


def test_leaves_topics_with_several_types(node, discover):
    discovery = discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    node.advertise("/scan", "sensor_msgs/msg/Range")
    discovery.scan()
    assert node.subscriptions == []


@pytest.mark.parametrize(
    ("publishers", "reliability", "durability"),
    [
        ([QoSProfile(depth=10)], ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
        ([QoSProfile(depth=10), qos_profile_sensor_data], ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
        ([LATCHED, LATCHED], ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL),
        # A transient local subscription would receive nothing from the volatile publisher
        ([LATCHED, QoSProfile(depth=10)], ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
    ],
)
def test_subscribes_with_qos_every_publisher_accepts(node, discover, publishers, reliability, durability):
    discovery = discover()
    for qos in publishers:
        node.advertise("/map", "nav_msgs/msg/OccupancyGrid", qos)
    discovery.scan()
    qos = subscribed(node)["/map"].qos
    assert (qos.reliability, qos.durability) == (reliability, durability)


def test_leaves_configured_topics_to_the_config(node, discover):
    discovery = discover(modules=[{"name": "scan", "module": "range", "topic": "scan", "entity_path": "my_scan"}])
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    assert discovery.modules == [] and len(node.subscriptions) == 1


def test_leaves_topics_whose_entity_path_the_config_uses(node, discover):
    discovery = discover(modules=[{"name": "range", "module": "range", "topic": "/range", "entity_path": "/scan"}])
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    assert discovery.modules == []
    assert ("warn", "[auto] not bridging /scan: entity path 'scan' is already in use") in node.log_messages


def test_tf_shares_its_entity_with_a_configured_module(node, discover):
    # As the joint_states module's docs advise, so the latest transform wins
    discovery = discover(
        modules=[{"name": "joints", "module": "joint_states", "topic": "/joint_states", "entity_path": "tf"}]
    )
    node.advertise("/tf", "tf2_msgs/msg/TFMessage")
    discovery.scan()
    assert discovery.modules[0].entity_path == "tf"


def test_bridges_only_included_topics_that_are_not_excluded(node, discover):
    discovery = discover(auto={"include": ["/camera/*", "/scan"], "exclude": ["*/depth/*"]})
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    node.advertise("/range", "sensor_msgs/msg/Range")
    node.advertise("/camera/front/image_raw", "sensor_msgs/msg/Image")
    node.advertise("/camera/depth/image_raw", "sensor_msgs/msg/Image")
    discovery.scan()
    assert set(subscribed(node)) == {"/scan", "/camera/front/image_raw"}


def test_a_topic_that_fails_to_bridge_does_not_stop_the_others(node, discover, monkeypatch):
    # As a module whose message package is not installed, e.g. nav2_msgs
    monkeypatch.setitem(MODULES_BY_TYPE, "missing_msgs/msg/Thing", {"module": "missing_msgs.module:Thing"})
    discovery = discover()
    node.advertise("/thing", "missing_msgs/msg/Thing")
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    discovery.scan()
    assert set(subscribed(node)) == {"/scan"}
    warnings = [message for level, message in node.log_messages if level == "warn"]
    assert warnings == ["[auto] not bridging /thing: ImportError: [/thing] No module named 'missing_msgs'"]


@pytest.mark.parametrize(("auto", "expected"), [(True, ["*"]), (False, None), ({"include": ["/a"]}, ["/a"])])
def test_auto_is_a_switch_or_its_filters(auto, expected):
    cfg = BridgeConfig(auto=auto)
    assert (cfg.auto.include if cfg.auto else None) == expected
