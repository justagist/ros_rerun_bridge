"""Robot model, joint states and odometry."""

import math
from pathlib import Path

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

DATA_DIR = Path(__file__).parent / "data"

URDF = f"""<robot name="tester">
  <link name="base">
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="arm">
    <visual><geometry><mesh filename="file://{DATA_DIR / "triangle_inches.dae"}" scale="2 2 2"/></geometry></visual>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>"""

BASE = "/robot/tester/visual_geometries/base/visual_0"
ARM = "/robot/tester/visual_geometries/arm/visual_0"


@pytest.fixture
def robot(node, build, context, recording):
    """A robot_description module that has received the test URDF."""
    build("robot_description", entity_path="robot")
    node.publish(String(data=URDF))
    return context.urdf_tree


def test_visual_geometry_is_logged_in_each_link_frame(robot, recording):
    assert recording.latest(BASE, "CoordinateFrame:frame") == ["base"]
    assert recording.latest(ARM, "CoordinateFrame:frame") == ["arm"]


def test_collision_geometry_is_not_logged(robot, recording):
    assert not [entity for entity in recording.entities() if "collision" in entity]


def test_joints_at_zero_are_logged_as_static_defaults(robot, recording):
    rows = recording.rows("/tf_static")
    assert all(row["static"] for row in rows)
    assert [row["Transform3D:child_frame"] for row in rows] == [["arm"]]


def test_collada_meshes_are_scaled_by_their_unit_and_urdf_scale(robot, recording):
    # The mesh is in inches (0.0254 m) and the URDF scales it by 2; the last scale logged wins
    assert recording.latest(ARM, "InstancePoses3D:scales") == [pytest.approx([0.0508] * 3)]


def test_other_geometry_is_not_rescaled(robot, recording):
    assert recording.latest(BASE, "InstancePoses3D:scales") is None


def test_robot_model_is_shared_with_other_modules(robot):
    assert [joint.name for joint in robot.joints()] == ["shoulder"]


def test_joint_states_move_joints_beyond_their_limits(robot, node, build, recording):
    build("joint_states", entity_path="tf")
    msg = JointState(name=["shoulder", "not_a_joint"], position=[2.0, 1.0])
    msg.header.stamp = TimeMsg(sec=3)
    node.publish(msg)

    rows = recording.rows("/tf")
    assert len(rows) == 1
    assert rows[0]["ros_time"] == 3_000_000_000
    assert rows[0]["Transform3D:parent_frame"] == ["base"]
    assert rows[0]["Transform3D:child_frame"] == ["arm"]
    # 2 rad about z, not clamped to the 1 rad limit
    assert rows[0]["Transform3D:quaternion"] == [pytest.approx([0.0, 0.0, math.sin(1.0), math.cos(1.0)])]


def test_joint_states_wait_for_the_robot_model(node, build, recording):
    build("joint_states", entity_path="tf")
    node.publish(JointState(name=["shoulder"], position=[0.5]))
    assert "/tf" not in recording.entities()
    assert node.log_messages[-1][0] == "warn"


def test_odometry_logs_velocities_as_scalars(node, build, recording):
    build("odometry", entity_path="odometry")
    msg = Odometry()
    msg.twist.twist.linear.x, msg.twist.twist.angular.z = 0.5, -0.2
    node.publish(msg)
    assert recording.rows("/odometry/vel")[0]["Scalars:scalars"] == pytest.approx([0.5])
    assert recording.rows("/odometry/ang_vel")[0]["Scalars:scalars"] == pytest.approx([-0.2])
