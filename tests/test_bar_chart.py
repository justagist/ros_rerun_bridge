"""Array fields drawn as bar charts."""

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import JointState


def build_bars(build, **params):
    return build(
        "bar_chart",
        entity_path="bars",
        msg_type="sensor_msgs.msg.JointState",
        params={"field": "effort", **params},
    )


def test_each_message_logs_its_array_as_bars(node, build, recording):
    build_bars(build, colour=[255, 128, 0, 255])
    node.publish(JointState(effort=[1.0, -2.0, 0.5]))
    node.publish(JointState(effort=[1.5, -1.0]))
    rows = recording.rows("/bars")
    # Logged as a 1D tensor
    assert [row["BarChart:values"][0]["buffer"] for row in rows] == [[1.0, -2.0, 0.5], [1.5, -1.0]]
    assert rows[0]["BarChart:color"] == [0xFF8000FF]


def test_bars_are_logged_at_their_stamp(node, build, recording):
    build_bars(build)
    msg = JointState(effort=[1.0])
    msg.header.stamp = TimeMsg(sec=5)
    node.publish(msg)
    assert recording.rows("/bars")[0]["ros_time"] == 5_000_000_000


@pytest.mark.parametrize(
    ("msg_type", "params", "error"),
    [
        (None, {"field": "effort"}, "needs `msg_type`"),
        ("sensor_msgs.msg.Range", {"field": "range"}, "field 'range' is float, not an array"),
        ("sensor_msgs.msg.JointState", {"field": "name"}, "is sequence<string>, not a number"),
    ],
)
def test_invalid_config_is_an_error(build, msg_type, params, error):
    with pytest.raises(ValueError, match=error):
        build("bar_chart", msg_type=msg_type, params=params)
