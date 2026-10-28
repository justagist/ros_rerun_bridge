"""Fields shown as discrete states over time."""

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, Int32, String

from ros_rerun_bridge.modules.states import state_text
from ros_rerun_bridge.utils.fields import is_single_value


def build_states(build, msg_type: str, field: str = "data", **params):
    return build("states", entity_path="mode", msg_type=msg_type, params={"field": field, **params})


def state_rows(recording) -> list:
    return [row for row in recording.rows("/mode") if not row["static"]]


def test_a_state_is_logged_only_when_the_value_changes(node, build, recording):
    build_states(build, "std_msgs.msg.Int32")
    for value in (0, 0, 1, 1, 0):
        node.publish(Int32(data=value))
    assert [row["StateChange:state"] for row in state_rows(recording)] == [["0"], ["1"], ["0"]]


@pytest.mark.parametrize(
    ("value", "ros_type", "text"),
    [
        (1, "float", "1"),
        (1.0, "float", "1"),
        # A float32 field's 0.3 is 0.30000001192092896 in Python
        (0.30000001192092896, "float", "0.3"),
        (0.3, "float", "0.3"),
        # Distinct values stay distinct, and large ones are not in exponent notation
        (1.0000001, "float", "1.0000001"),
        (1.0000002, "float", "1.0000002"),
        (1000000.0, "double", "1000000"),
        (1000000, "double", "1000000"),
        (2.0, "int32", "2"),
        (True, "boolean", "true"),
        ("idle", "string", "idle"),
        (1, "string", "1"),
    ],
)
def test_values_equal_in_the_field_type_give_the_same_state_text(value, ros_type, text):
    assert state_text(value, ros_type) == text


@pytest.mark.parametrize(
    ("value", "ros_type", "error"),
    [
        # YAML reads an unquoted on as True
        (True, "string", "quote it"),
        ("on", "boolean", "is not a bool"),
        ("fast", "double", "is not a number"),
        (True, "int32", "is not a number"),
        (1.5, "int32", "is not an integer"),
    ],
)
def test_config_values_the_field_could_never_hold_are_an_error(value, ros_type, error):
    with pytest.raises(ValueError, match=error):
        state_text(value, ros_type)


@pytest.mark.parametrize("ros_type", ["string", "string<22>", "wstring<5>"])
def test_bounded_and_wide_string_fields_are_single_values(ros_type):
    # As ROS spells them, e.g. test_msgs/Strings.bounded_string_value is string<22>
    assert is_single_value(ros_type)


def test_float32_fields_match_config_values(node, build, recording):
    # A float32 1.0 must give the same state as the config's `value: 1`
    build_states(build, "std_msgs.msg.Float32", states=[{"value": 1, "label": "on"}])
    node.publish(Float32(data=1.0))
    assert state_rows(recording)[0]["StateChange:state"] == ["1"]


def test_states_are_logged_at_their_stamp_or_receive_time(node, build, recording):
    build("states", entity_path="sensor", msg_type="sensor_msgs.msg.Range", params={"field": "radiation_type"})
    stamped = Range(radiation_type=Range.INFRARED)
    stamped.header.stamp = TimeMsg(sec=5)
    node.publish(stamped)
    build_states(build, "std_msgs.msg.String")
    node.now_ns = 9_000_000_000
    node.publish(String(data="idle"))

    assert recording.rows("/sensor")[0]["ros_time"] == 5_000_000_000
    assert state_rows(recording)[0]["ros_time"] == 9_000_000_000


def test_state_labels_and_colours_are_logged_once_as_static(node, build, recording):
    states = [
        {"value": False, "label": "off", "colour": [60, 60, 60, 255]},
        {"value": True, "label": "on", "colour": [220, 40, 40, 255]},
    ]
    build_states(build, "std_msgs.msg.Bool", states=states)
    node.publish(Bool(data=True))
    [config] = [row for row in recording.rows("/mode") if row["static"]]
    assert config["StateConfiguration:values"] == ["false", "true"]
    assert config["StateConfiguration:labels"] == ["off", "on"]
    assert config["StateConfiguration:colors"] == [0x3C3C3CFF, 0xDC2828FF]


def test_states_without_a_label_show_their_value(node, build, recording):
    build_states(build, "std_msgs.msg.Int32", states=[{"value": 2}])
    [config] = [row for row in recording.rows("/mode") if row["static"]]
    assert config["StateConfiguration:labels"] == ["2"]
    assert config.get("StateConfiguration:colors") is None


@pytest.mark.parametrize(
    ("msg_type", "params", "error"),
    [
        (None, {"field": "data"}, "needs `msg_type`"),
        ("sensor_msgs.msg.JointState", {"field": "position"}, "is sequence<double>, not a number, bool or string"),
        ("sensor_msgs.msg.Range", {"field": "header"}, "is std_msgs/Header, not a number, bool or string"),
        ("std_msgs.msg.Int32", {"field": "value"}, "Int32 has no field 'value'"),
        (
            "std_msgs.msg.Int32",
            {"field": "data", "states": [{"value": 0, "colour": [0, 0, 0, 255]}, {"value": 1}]},
            "a colour for every state, or for none",
        ),
        ("std_msgs.msg.String", {"field": "data", "states": [{"value": True}]}, "quote it"),
    ],
)
def test_invalid_config_is_an_error(build, msg_type, params, error):
    with pytest.raises(ValueError, match=error):
        build("states", msg_type=msg_type, params=params)


def test_states_earlier_than_the_latest_are_logged_as_they_are(node, build, recording):
    # E.g. a bag replayed from its start: deduplicating against a later state would leave them out
    build("states", entity_path="mode", msg_type="sensor_msgs.msg.Range", params={"field": "radiation_type"})
    for sec, value in [(10, 0), (20, 1), (5, 1), (21, 1), (22, 0)]:
        msg = Range(radiation_type=value)
        msg.header.stamp = TimeMsg(sec=sec)
        node.publish(msg)
    rows = [(row["ros_time"] // 1_000_000_000, row["StateChange:state"]) for row in state_rows(recording)]
    assert sorted(rows) == [(5, ["1"]), (10, ["0"]), (20, ["1"]), (22, ["0"])]
