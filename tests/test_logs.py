"""ROS log entries and diagnostics, shown as text logs and states."""

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Log


def log_rows(recording, entity: str) -> list:
    return [(row["TextLog:level"], row["TextLog:text"]) for row in recording.rows(entity) if "TextLog:text" in row]


def test_each_loggers_entries_are_logged_on_its_own_entity(node, build, recording):
    build("rosout", entity_path="logs")
    node.publish(Log(name="planner", level=Log.INFO, msg="planning"))
    node.publish(Log(name="arm.controller", level=Log.WARN, msg="slow"))
    assert log_rows(recording, "/logs/planner") == [(["INFO"], ["planning"])]
    assert log_rows(recording, "/logs/arm/controller") == [(["WARN"], ["slow"])]


@pytest.mark.parametrize(
    ("level", "rerun"),
    # Levels between or beyond ROS's named ones are taken as the named one below them
    [(Log.DEBUG, "DEBUG"), (Log.FATAL, "CRITICAL"), (25, "INFO"), (0, "DEBUG"), (90, "CRITICAL")],
)
def test_ros_levels_map_to_rerun_levels(node, build, recording, level, rerun):
    build("rosout", entity_path="logs")
    node.publish(Log(name="planner", level=level, msg="planning"))
    assert log_rows(recording, "/logs/planner") == [([rerun], ["planning"])]


def test_entries_without_a_logger_name_are_logged_on_the_entity_path(node, build, recording):
    build("rosout", entity_path="logs")
    node.publish(Log(name="", level=Log.INFO, msg="unnamed"))
    assert log_rows(recording, "/logs") == [(["INFO"], ["unnamed"])]


def test_entries_below_the_minimum_level_are_left_out(node, build, recording):
    build("rosout", entity_path="logs", params={"min_level": "warn"})
    node.publish(Log(name="planner", level=Log.INFO, msg="planning"))
    node.publish(Log(name="planner", level=Log.ERROR, msg="no plan"))
    assert log_rows(recording, "/logs/planner") == [(["ERROR"], ["no plan"])]


def test_entries_are_logged_at_their_stamp(node, build, recording):
    build("rosout", entity_path="logs")
    node.publish(Log(name="planner", stamp=TimeMsg(sec=5), msg="planning"))
    assert recording.rows("/logs/planner")[0]["ros_time"] == 5_000_000_000


def diagnostics(*statuses: tuple[str, bytes, str]) -> DiagnosticArray:
    return DiagnosticArray(
        status=[DiagnosticStatus(name=name, level=level, message=message) for name, level, message in statuses]
    )


def state_rows(recording, entity: str) -> list:
    return [row["StateChange:state"] for row in recording.rows(entity) if "StateChange:state" in row]


def test_a_status_is_logged_only_when_its_level_or_message_changes(node, build, recording):
    build("diagnostics", entity_path="diagnostics")
    for level, message in [
        (DiagnosticStatus.OK, "ok"),
        (DiagnosticStatus.OK, "ok"),
        (DiagnosticStatus.OK, "warming up"),
        (DiagnosticStatus.ERROR, "too hot"),
        (DiagnosticStatus.ERROR, "too hot"),
    ]:
        node.publish(diagnostics(("motor", level, message)))
    assert state_rows(recording, "/diagnostics/motor") == [["OK"], ["ERROR"]]
    assert log_rows(recording, "/diagnostics/motor") == [
        (["INFO"], ["ok"]),
        (["INFO"], ["warming up"]),
        (["ERROR"], ["too hot"]),
    ]


def test_aggregated_status_names_nest_as_a_tree(node, build, recording):
    build("diagnostics", entity_path="diagnostics")
    node.publish(
        diagnostics(
            ("/Robot/Arm/Motor", DiagnosticStatus.WARN, "hot"),
            ("controller_manager: Controller Activity", DiagnosticStatus.STALE, "no data"),
        )
    )
    assert state_rows(recording, "/diagnostics/Robot/Arm/Motor") == [["WARN"]]
    # A stale status has no log level of its own in Rerun
    assert log_rows(recording, r"/diagnostics/controller_manager\:\ Controller\ Activity") == [(["WARN"], ["no data"])]


def test_each_status_is_given_the_level_colours_once(node, build, recording):
    build("diagnostics", entity_path="diagnostics")
    node.publish(diagnostics(("motor", DiagnosticStatus.OK, "ok")))
    node.publish(diagnostics(("motor", DiagnosticStatus.WARN, "hot")))
    [config] = [row for row in recording.rows("/diagnostics/motor") if row["static"]]
    assert config["StateConfiguration:values"] == ["OK", "WARN", "ERROR", "STALE"]
    assert config["StateConfiguration:colors"] == [0x28AA46FF, 0xF0B400FF, 0xDC2828FF, 0x828282FF]


def test_an_unknown_level_is_shown_as_its_number_and_logged_as_a_warning(node, build, recording):
    build("diagnostics", entity_path="diagnostics")
    node.publish(diagnostics(("motor", bytes([7]), "odd")))
    assert state_rows(recording, "/diagnostics/motor") == [["7"]]
    assert log_rows(recording, "/diagnostics/motor") == [(["WARN"], ["odd"])]


def test_statuses_earlier_than_the_latest_are_logged_as_they_are(node, build, recording):
    build("diagnostics", entity_path="diagnostics")
    for sec, level in [(10, DiagnosticStatus.OK), (20, DiagnosticStatus.ERROR), (5, DiagnosticStatus.ERROR)]:
        msg = diagnostics(("motor", level, "status"))
        msg.header.stamp = TimeMsg(sec=sec)
        node.publish(msg)
    rows = [row for row in recording.rows("/diagnostics/motor") if "StateChange:state" in row]
    assert [(row["ros_time"] // 1_000_000_000, row["StateChange:state"]) for row in rows] == [
        (5, ["ERROR"]),
        (10, ["OK"]),
        (20, ["ERROR"]),
    ]
