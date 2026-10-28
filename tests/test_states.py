"""Fields shown as discrete states over time."""

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, Int32, String

from ros_rerun_bridge.modules.states import state_text


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
    ("value", "text"), [(1, "1"), (1.0, "1"), (0.25, "0.25"), (True, "true"), (False, "false"), ("idle", "idle")]
)
def test_values_and_config_values_give_the_same_state_text(value, text):
    assert state_text(value) == text


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
    ],
)
def test_invalid_config_is_an_error(build, msg_type, params, error):
    with pytest.raises(ValueError, match=error):
        build("states", msg_type=msg_type, params=params)
