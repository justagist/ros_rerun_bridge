"""Automatic discovery picks a module per topic, subscribes with QoS its publishers accept, and leaves configured
topics and entity paths alone."""

import pytest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pydantic import ValidationError
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from ros_rerun_bridge.builder import BridgeBuilder
from ros_rerun_bridge.config import AutoConfig, BridgeConfig
from ros_rerun_bridge.discovery import MODULES_BY_TYPE, TopicDiscovery, suggest
from ros_rerun_bridge.modules.compressed_image import CompressedImageModule
from ros_rerun_bridge.modules.image import ImageModule
from ros_rerun_bridge.modules.joint_states import JointStatesModule
from ros_rerun_bridge.modules.laserscan import LaserScanModule
from ros_rerun_bridge.modules.range import RangeModule
from ros_rerun_bridge.modules.tf import TFModule

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
AUTO = AutoConfig()
STALE_SCANS = AUTO.stale_scans


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


def deliver(node, topic: str, msg) -> None:
    """Deliver a message to every subscription to a topic, as the executor would."""
    for subscription in [s for s in node.subscriptions if s.topic == topic]:
        subscription.callback(msg)


@pytest.mark.parametrize(
    ("topic", "msg_type", "module"),
    [
        ("/tf", "tf2_msgs/msg/TFMessage", "tf"),
        ("/robot_description", "std_msgs/msg/String", "robot_description"),
        ("/camera/image_raw", "sensor_msgs/msg/Image", "image"),
        ("/camera/depth/image_rect_raw", "sensor_msgs/msg/Image", "depth_image"),
        ("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage", "compressed_image"),
        ("/camera/depth/image_raw/compressedDepth", "sensor_msgs/msg/CompressedImage", "compressed_depth_image"),
    ],
)
def test_picks_the_module_from_the_type_and_name(topic, msg_type, module):
    assert [entry["module"] for entry in suggest(topic, msg_type, {topic: [msg_type]}, AUTO)] == [module]


def test_tf_static_is_logged_as_static():
    [entry] = suggest("/tf_static", "tf2_msgs/msg/TFMessage", {}, AUTO)
    assert (entry["module"], entry["params"]["static"], entry["entity_path"]) == ("tf", True, "tf_static")


@pytest.mark.parametrize("topic", ["/tf", "/tf_static"])
def test_tf_frames_are_drawn_as_axes_without_names(node, discover, recording, topic):
    discovery = discover()
    node.advertise(topic, "tf2_msgs/msg/TFMessage")
    discovery.scan()
    transform = TransformStamped(child_frame_id="tool")
    transform.header.frame_id = "world"
    node.publish(TFMessage(transforms=[transform]))
    [axes] = recording.rows("/frames/tool")
    assert axes["TransformAxes3D:axis_length"] == [pytest.approx(0.1)]
    assert axes["TransformAxes3D:show_frame"] == [False]


@pytest.mark.parametrize(
    ("frames", "length", "names"),
    [({"axis_length": 0.3, "frame_names": True}, 0.3, True)],
)
def test_auto_frames_set_how_tf_frames_are_drawn(node, discover, recording, frames, length, names):
    discovery = discover(auto={"frames": frames})
    node.advertise("/tf", "tf2_msgs/msg/TFMessage")
    discovery.scan()
    node.publish(TFMessage(transforms=[TransformStamped(child_frame_id="tool")]))
    [axes] = recording.rows("/frames/tool")
    assert axes["TransformAxes3D:axis_length"] == [pytest.approx(length)]
    assert axes["TransformAxes3D:show_frame"] == [names]


def test_auto_frames_without_an_axis_length_draw_no_axes(node, discover, recording):
    discovery = discover(auto={"frames": {"axis_length": None}})
    node.advertise("/tf", "tf2_msgs/msg/TFMessage")
    discovery.scan()
    node.publish(TFMessage(transforms=[TransformStamped(child_frame_id="tool")]))
    assert "/tf" in recording.entities() and "/frames/tool" not in recording.entities()


URDF = """<robot name="arm">
  <link name="base"/><link name="arm"/>
  <joint name="elbow" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>"""


def test_joint_states_move_the_robot_and_plot_positions_only(node, discover, recording):
    discovery = discover()
    node.advertise("/robot_description", "std_msgs/msg/String", LATCHED)
    node.advertise("/joint_states", "sensor_msgs/msg/JointState")
    discovery.scan()
    deliver(node, "/robot_description", String(data=URDF))
    deliver(node, "/joint_states", JointState(name=["elbow"], position=[0.5], effort=[2.0]))
    assert [row["Transform3D:child_frame"] for row in recording.rows("/tf")] == [["arm"]]
    assert {entity for entity in recording.entities() if entity.startswith("/joint_states/")} == {
        "/joint_states/position/elbow"
    }


def test_joint_state_settings_apply_to_the_topics_they_name(node, discover):
    joint_states = {"topics": ["/joint_states"], "prefer": "joint_states", "plot": ["velocity", "effort"]}
    discovery = discover(auto={"joint_states": joint_states})
    node.advertise("/joint_states", "sensor_msgs/msg/JointState")
    # e.g. commands, which would draw the robot at its targets
    node.advertise("/joint_command", "sensor_msgs/msg/JointState")
    discovery.scan()
    [module] = discovery.modules
    assert module.params.model_dump(include={"prefer", "plot", "plot_entity_path"}) == {
        "prefer": "joint_states",
        "plot": ["velocity", "effort"],
        "plot_entity_path": "joint_states",
    }
    assert "/joint_command" not in subscribed(node)


def test_each_joint_state_topic_moves_the_robot(node, discover):
    discovery = discover()
    node.advertise("/arm/joint_states", "sensor_msgs/msg/JointState")
    node.advertise("/gripper/joint_states", "sensor_msgs/msg/JointState")
    discovery.scan()
    motion = [module.entity_path for module in discovery.modules if isinstance(module, JointStatesModule)]
    assert motion == ["tf", "tf"]


def test_joint_states_move_the_robot_on_a_configured_tf_entity(node, discover):
    discovery = discover(modules=[{"name": "tf", "module": "tf", "topic": "/tf", "entity_path": "transforms"}])
    node.advertise("/joint_states", "sensor_msgs/msg/JointState")
    # Any other topic there could clear the frame relations
    node.advertise("/transforms", "sensor_msgs/msg/Range")
    discovery.scan()
    motion = [module.entity_path for module in discovery.modules if isinstance(module, JointStatesModule)]
    assert motion == ["transforms"]
    assert "/transforms" not in subscribed(node)


def test_a_topic_whose_second_module_fails_to_build_leaves_no_subscription(node, discover, monkeypatch):
    def fail(*args, **kwargs):
        raise RuntimeError("broken")

    monkeypatch.setattr(JointStatesModule, "__init__", fail)
    discovery = discover()
    node.advertise("/joint_states", "sensor_msgs/msg/JointState")
    discovery.scan()
    assert discovery.modules == [] and node.subscriptions == []


@pytest.mark.parametrize(
    ("topic", "msg_type"),
    [
        ("/robot1/tf", "tf2_msgs/msg/TFMessage"),
        ("/robot1/robot_description", "std_msgs/msg/String"),
        ("/chatter", "std_msgs/msg/String"),
        ("/_hidden/scan", "sensor_msgs/msg/LaserScan"),
        ("/scan/_raw", "sensor_msgs/msg/LaserScan"),
    ],
)
def test_leaves_topics_no_module_draws_or_that_are_hidden_or_namespaced_robots(topic, msg_type):
    assert suggest(topic, msg_type, {topic: [msg_type]}, AUTO) == []


def test_leaves_image_transport_copies_of_a_raw_image():
    graph = {"/camera/image_raw": ["sensor_msgs/msg/Image"]}
    assert suggest("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage", graph, AUTO) == []
    assert suggest("/camera/image_raw/compressedDepth", "sensor_msgs/msg/CompressedImage", graph, AUTO) == []


@pytest.mark.parametrize(
    ("config", "expected"),
    [
        # The raw image is filtered out, so its compressed copy is the only one shown
        ({"auto": {"include": ["*/compressed"]}}, {"/camera/image_raw/compressed"}),
        # The raw image is configured, so it is bridged and its copy is not
        (
            {"modules": [{"name": "raw", "module": "image", "topic": "/camera/image_raw", "entity_path": "raw"}]},
            {"/camera/image_raw"},
        ),
    ],
)
def test_bridges_a_compressed_image_unless_its_raw_image_is_bridged(node, discover, config, expected):
    discovery = discover(**config)
    node.advertise("/camera/image_raw", "sensor_msgs/msg/Image")
    node.advertise("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage")
    discovery.scan()
    assert set(subscribed(node)) == expected


def test_leaves_the_robot_model_to_a_configured_one(node, discover):
    # The bridge holds one robot model: the namespaced robot's must not be replaced
    discovery = discover(
        modules=[
            {"name": "robot", "module": "robot_description", "topic": "/robot1/robot_description", "entity_path": "r"}
        ]
    )
    node.advertise("/robot_description", "std_msgs/msg/String", LATCHED)
    discovery.scan()
    assert discovery.modules == []


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


def test_scans_on_a_timer_as_often_as_auto_sets(node, discover):
    discover(auto={"scan_period": 0.5})
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    node.timers[-1]()
    assert "/scan" in subscribed(node)
    assert node.timer_periods[-1] == 0.5


def test_single_marker_topics_subscribe_with_the_marker_type(node, discover):
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


def test_leaves_joint_states_whose_plots_entity_path_the_config_uses(node, discover):
    discovery = discover(
        modules=[{"name": "range", "module": "range", "topic": "/range", "entity_path": "joint_states"}]
    )
    node.advertise("/joint_states", "sensor_msgs/msg/JointState")
    discovery.scan()
    assert discovery.modules == []
    assert (
        "warn",
        "[auto] not bridging /joint_states: entity path 'joint_states' is already in use",
    ) in node.log_messages


def test_tf_shares_its_entity_with_a_configured_module(node, discover):
    # As the joint_states module's docs advise, so that a joint on both is one frame relation
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


@pytest.mark.parametrize(("auto", "expected"), [(True, ["*"]), (False, None)])
def test_auto_can_be_a_switch(auto, expected):
    cfg = BridgeConfig(auto=auto)
    assert (cfg.auto.include if cfg.auto else None) == expected


def drop_after_grace(node, discovery, topic: str) -> None:
    """Withdraw a topic's publishers and scan until discovery drops it."""
    node.withdraw(topic)
    for _ in range(STALE_SCANS):
        discovery.scan()


def test_keeps_a_topic_whose_publishers_return_within_the_grace_period(node, discover):
    # A restarting publisher keeps its topic's module
    discovery = discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    [module] = discovery.modules
    node.withdraw("/scan")
    for _ in range(STALE_SCANS - 1):
        discovery.scan()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    assert discovery.modules == [module] and len(node.subscriptions) == 1


def test_a_publisher_returning_restarts_the_count_of_scans_without_one(node, discover):
    discovery = discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    [module] = discovery.modules
    for _ in range(2):
        node.withdraw("/scan")
        for _ in range(STALE_SCANS - 1):
            discovery.scan()
        node.advertise("/scan", "sensor_msgs/msg/LaserScan")
        discovery.scan()
    # Kept, not dropped and built again
    assert discovery.modules == [module]


def test_drops_a_topic_once_its_publishers_have_gone_and_bridges_it_again_when_they_return(node, discover):
    discovery = discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    drop_after_grace(node, discovery, "/scan")
    assert discovery.modules == [] and node.subscriptions == []
    assert ("info", "[auto] dropped /scan: it has no publishers") in node.log_messages
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    assert set(subscribed(node)) == {"/scan"}


def test_clears_what_a_dropped_stream_drew_from_when_its_publishers_went(node, discover, recording):
    discovery = discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    node.withdraw("/scan")
    went_ns = node.now_ns
    discovery.scan()
    node.now_ns += 5_000_000_000
    for _ in range(STALE_SCANS - 1):
        discovery.scan()
    [clear] = [row for row in recording.rows("/scan") if "Clear:is_recursive" in row]
    assert clear["Clear:is_recursive"] == [False] and clear["ros_time"] == went_ns


@pytest.mark.parametrize(
    ("topic", "msg_type"),
    [
        # Every frame relation shares the tf entity, joints moved by joint states included
        ("/tf", "tf2_msgs/msg/TFMessage"),
        ("/joint_states", "sensor_msgs/msg/JointState"),
        # Plots and logs show history, which stays true
        ("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"),
    ],
)
def test_keeps_what_dropped_tf_or_history_drew(node, discover, recording, topic, msg_type):
    discovery = discover()
    node.advertise(topic, msg_type)
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    drop_after_grace(node, discovery, topic)
    assert discovery.modules[0].entity_path == "scan"
    assert all("Clear:is_recursive" not in row for path in (topic, "/tf") for row in recording.rows(path))


def test_never_drops_a_latched_topic(node, discover, recording):
    # Its last message stays true, e.g. a map, and its module keeps what it needs to update it, e.g. a robot model's
    # links to replace
    discovery = discover()
    node.advertise("/robot_description", "std_msgs/msg/String", LATCHED)
    discovery.scan()
    [module] = discovery.modules
    drop_after_grace(node, discovery, "/robot_description")
    assert discovery.modules == [module] and len(node.subscriptions) == 1
    assert all("Clear:is_recursive" not in row for row in recording.rows("/robot_description"))


def test_subscribes_again_keeping_the_module_when_its_publishers_qos_changed(node, discover):
    discovery = discover()
    node.advertise("/map", "nav_msgs/msg/OccupancyGrid", LATCHED)
    discovery.scan()
    [module] = discovery.modules
    # A transient local subscription receives nothing from the volatile publisher
    node.advertise("/map", "nav_msgs/msg/OccupancyGrid", QoSProfile(depth=10))
    discovery.scan()
    # The same module keeps its state, e.g. markers to delete or expire later
    assert discovery.modules == [module]
    [subscription] = node.subscriptions
    assert subscription.qos.durability == DurabilityPolicy.VOLATILE
    assert ("info", "[auto] subscribed to /map again: its publishers' QoS changed") in node.log_messages


def test_rebuilds_a_topic_whose_type_changed(node, discover):
    discovery = discover()
    node.advertise("/sensor", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    node.topics["/sensor"] = ["sensor_msgs/msg/Range"]
    discovery.scan()
    assert [type(m) for m in discovery.modules] == [RangeModule] and len(node.subscriptions) == 1


def test_drops_a_compressed_image_once_its_raw_image_is_published(node, discover, recording):
    discovery = discover()
    node.advertise("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage")
    discovery.scan()
    node.advertise("/camera/image_raw", "sensor_msgs/msg/Image")
    discovery.scan()
    assert set(subscribed(node)) == {"/camera/image_raw"}
    assert any("Clear:is_recursive" in row for row in recording.rows("/camera/image_raw/compressed"))


def test_tries_a_topic_that_failed_to_bridge_again_once_it_returns(node, discover, monkeypatch):
    monkeypatch.setitem(MODULES_BY_TYPE, "sensor_msgs/msg/LaserScan", {"module": "missing_msgs.module:Thing"})
    discovery = discover()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    discovery.scan()
    assert node.subscriptions == []
    # As when the missing package is installed and the publisher restarted
    monkeypatch.undo()
    node.withdraw("/scan")
    discovery.scan()
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    assert set(subscribed(node)) == {"/scan"}


def test_a_topic_is_stale_after_as_many_scans_as_auto_sets(node, discover):
    discovery = discover(auto={"stale_scans": 1})
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    node.withdraw("/scan")
    discovery.scan()
    assert node.subscriptions == []


def test_never_stale_topics_stay_bridged_without_publishers(node, discover, recording):
    discovery = discover(auto={"never_stale": ["/camera/*"]})
    node.advertise("/camera/image_raw", "sensor_msgs/msg/Image")
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    node.withdraw("/camera/image_raw")
    node.withdraw("/scan")
    for _ in range(STALE_SCANS + 1):
        discovery.scan()
    assert set(subscribed(node)) == {"/camera/image_raw"}
    assert all("Clear:is_recursive" not in row for row in recording.rows("/camera/image_raw"))


def test_clears_what_the_old_type_drew_when_a_topic_changes_type(node, discover, recording):
    discovery = discover()
    node.advertise("/sensor", "sensor_msgs/msg/PointCloud2")
    discovery.scan()
    node.topics["/sensor"] = ["sensor_msgs/msg/Image"]
    changed_ns = node.now_ns
    discovery.scan()
    [clear] = [row for row in recording.rows("/sensor") if "Clear:is_recursive" in row]
    assert clear["ros_time"] == changed_ns


def test_keeps_a_compressed_image_while_its_raw_image_is_not_shown(node, discover):
    # The raw image's entity path is taken, so it is not bridged
    discovery = discover(
        modules=[{"name": "other", "module": "range", "topic": "/range", "entity_path": "camera/image_raw"}]
    )
    node.advertise("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage")
    discovery.scan()
    node.advertise("/camera/image_raw", "sensor_msgs/msg/Image")
    discovery.scan()
    assert set(subscribed(node)) == {"/range", "/camera/image_raw/compressed"}


def test_clears_each_marker_of_a_dropped_marker_topic(node, discover, recording):
    discovery = discover()
    node.advertise("/markers", "visualization_msgs/msg/MarkerArray")
    discovery.scan()
    marker = Marker(ns="ns", id=0, type=Marker.CUBE, action=Marker.ADD)
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    node.publish(MarkerArray(markers=[marker]))
    drop_after_grace(node, discovery, "/markers")
    assert recording.latest("/markers/ns/0", "Boxes3D:half_sizes") == []


class CustomScan(LaserScanModule):
    """A module of the user's own."""


def test_shows_a_type_with_a_module_of_your_own(node, discover):
    discovery = discover(auto={"modules": {"sensor_msgs/msg/LaserScan": f"{__name__}:CustomScan"}})
    node.advertise("/scan", "sensor_msgs/msg/LaserScan")
    discovery.scan()
    assert [type(m) for m in discovery.modules] == [CustomScan]


def test_shows_a_type_no_built_in_module_draws_alone_with_a_mapped_one_and_its_params(node, discover, recording):
    discovery = discover(
        auto={
            "modules": {"nav_msgs/msg/Odometry": {"module": "scalars", "params": {"fields": ["twist.twist.linear.x"]}}}
        }
    )
    node.advertise("/odom", "nav_msgs/msg/Odometry")
    discovery.scan()
    odometry = Odometry()
    odometry.twist.twist.linear.x = 0.5
    node.publish(odometry)
    rows = recording.rows("/odom/twist/twist/linear/x")
    assert [row["Scalars:scalars"] for row in rows if not row["static"]] == [[0.5]]


def test_a_mapped_module_replaces_the_built_in_choice(node, discover):
    # Not depth_image, which the topic's name would pick
    discovery = discover(auto={"modules": {"sensor_msgs/msg/Image": "image"}})
    node.advertise("/camera/depth/image_raw", "sensor_msgs/msg/Image")
    discovery.scan()
    assert [type(m) for m in discovery.modules] == [ImageModule]


def test_a_mapped_module_replaces_every_built_in_module_of_the_type(node, discover):
    discovery = discover(auto={"modules": {"sensor_msgs/msg/JointState": "joint_states"}})
    node.advertise("/joint_states", "sensor_msgs/msg/JointState")
    discovery.scan()
    assert [type(m) for m in discovery.modules] == [JointStatesModule]


@pytest.mark.parametrize(
    ("modules", "error", "match"),
    [
        ({"sensor_msgs/msg/LaserScan": "laserscn"}, ValueError, r"\[auto.modules: sensor_msgs/msg/LaserScan\] Invalid"),
        ({"sensor_msgs/msg/LaserScan": {"module": "laserscan", "params": {"mood": "x"}}}, ValueError, "mood"),
        ({"missing_msgs/msg/Thing": "laserscan"}, ImportError, "missing_msgs"),
    ],
)
def test_a_mapped_module_that_cannot_be_used_stops_the_bridge_at_start(discover, modules, error, match):
    with pytest.raises(error, match=match):
        discover(auto={"modules": modules})


def test_a_mapping_must_name_a_message_type():
    with pytest.raises(ValidationError, match="'Odometry' is not a message type"):
        BridgeConfig(auto={"modules": {"Odometry": "scalars"}})


class CustomTF(TFModule):
    """A TF module of the user's own."""


def test_a_tf_mapping_leaves_the_root_tf_topics_to_the_bridge(node, discover):
    # The scene's frames and robot model depend on them, e.g. /tf_static's transforms being static
    discovery = discover(auto={"modules": {"tf2_msgs/msg/TFMessage": f"{__name__}:CustomTF"}})
    node.advertise("/tf_static", "tf2_msgs/msg/TFMessage", LATCHED)
    node.advertise("/robot1/tf", "tf2_msgs/msg/TFMessage")
    discovery.scan()
    modules = {module.entity_path: module for module in discovery.modules}
    assert type(modules["tf_static"]) is TFModule
    assert type(modules["robot1/tf"]) is CustomTF


@pytest.mark.parametrize(
    ("modules", "match"),
    [
        ({"nav_msgs/msg/Odomtry": "scalars"}, "Odomtry"),
        ({"sensor_msgs/msg/LaserScan": "ros_rerun_bridge.modules.laserscan:LaserScanModul"}, "LaserScanModul"),
    ],
)
def test_a_misspelled_class_stops_the_bridge_at_start(discover, modules, match):
    with pytest.raises(ImportError, match=match):
        discover(auto={"modules": modules})
