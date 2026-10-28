"""Poses, pose arrays and wrenches."""

import math

import numpy as np
import pytest
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, WrenchStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation


def yawed(x: float, y: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def test_poses_are_drawn_as_axes_in_their_own_frame(node, build, recording):
    build("pose", entity_path="goal", params={"axis_length": 0.5})
    msg = PoseStamped(pose=yawed(1.0, 2.0, math.pi / 2))
    msg.header.frame_id = "map"
    node.publish(msg)

    row = recording.rows("/goal")[0]
    assert row["Transform3D:parent_frame"] == ["map"]
    assert row["Transform3D:child_frame"] == ["pose:goal"]
    assert row["Transform3D:translation"] == [[1.0, 2.0, 0.0]]
    assert row["Transform3D:quaternion"] == [pytest.approx([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])]
    assert row["TransformAxes3D:axis_length"] == pytest.approx([0.5])
    assert row["CoordinateFrame:frame"] == ["pose:goal"]
    assert "/goal/covariance" not in recording.entities()


def test_an_all_zero_orientation_is_treated_as_identity(node, build, recording):
    build("pose", entity_path="goal")
    msg = PoseStamped()
    # ROS quaternions default to identity: hand-written publishers often leave all four at 0 instead
    msg.pose.orientation.w = 0.0
    node.publish(msg)
    assert recording.rows("/goal")[0]["Transform3D:quaternion"] == [[0.0, 0.0, 0.0, 1.0]]


def covariance(position: np.ndarray) -> list[float]:
    full = np.zeros((6, 6))
    full[:3, :3] = position
    return full.flatten().tolist()


@pytest.mark.parametrize(
    "position",
    [
        np.diag([0.04, 0.01, 0.0025]),
        # Correlated x and y: the ellipsoid is turned in the plane
        np.array([[0.05, 0.03, 0.0], [0.03, 0.05, 0.0], [0.0, 0.0, 0.01]]),
    ],
    ids=["axis aligned", "correlated"],
)
def test_the_position_covariance_is_an_ellipsoid_n_sigma_across(node, build, recording, position):
    build("pose", entity_path="robot", msg_type="geometry_msgs.msg.PoseWithCovarianceStamped", params={"n_sigma": 2})
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose = yawed(1.0, 2.0, 0.3)
    msg.pose.covariance = covariance(position)
    node.publish(msg)

    row = recording.rows("/robot/covariance")[0]
    assert row["Ellipsoids3D:centers"] == [[1.0, 2.0, 0.0]]
    assert row["CoordinateFrame:frame"] == ["map"]
    half_sizes = np.array(row["Ellipsoids3D:half_sizes"][0])
    rotation = Rotation.from_quat(row["Ellipsoids3D:quaternions"][0]).as_matrix()
    # The ellipsoid's axes, rotated back, give the covariance it was drawn from
    np.testing.assert_allclose(rotation @ np.diag(half_sizes**2) @ rotation.T, 4 * position, atol=1e-6)


def test_odometry_poses_and_their_covariance_are_drawn(node, build, recording):
    build("pose", entity_path="odom", msg_type="nav_msgs.msg.Odometry")
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.pose.pose = yawed(3.0, 0.0, 0.0)
    msg.pose.covariance = covariance(np.diag([0.01, 0.01, 0.01]))
    node.publish(msg)
    assert recording.rows("/odom")[0]["Transform3D:translation"] == [[3.0, 0.0, 0.0]]
    assert recording.rows("/odom/covariance")[0]["Ellipsoids3D:half_sizes"] == [pytest.approx([0.2, 0.2, 0.2])]


def test_the_covariance_can_be_left_out(node, build, recording):
    build("pose", entity_path="robot", msg_type="geometry_msgs.msg.PoseWithCovarianceStamped", params={"n_sigma": None})
    node.publish(PoseWithCovarianceStamped())
    assert "/robot/covariance" not in recording.entities()


def test_pose_arrays_draw_an_arrow_along_each_pose_x_axis(node, build, recording):
    build("pose_array", entity_path="particles", params={"arrow_length": 0.5})
    msg = PoseArray(poses=[yawed(1.0, 0.0, 0.0), yawed(0.0, 1.0, math.pi / 2)])
    msg.header.frame_id = "map"
    node.publish(msg)

    row = recording.rows("/particles")[0]
    np.testing.assert_allclose(row["Arrows3D:origins"], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    np.testing.assert_allclose(row["Arrows3D:vectors"], [[0.5, 0.0, 0.0], [0.0, 0.5, 0.0]], atol=1e-6)
    assert row["CoordinateFrame:frame"] == ["map"]


def test_an_empty_pose_array_clears_the_arrows(node, build, recording):
    build("pose_array", entity_path="particles")
    node.publish(PoseArray(poses=[yawed(1.0, 0.0, 0.0)]))
    node.publish(PoseArray())
    assert recording.latest("/particles", "Arrows3D:vectors") == []


def test_wrenches_are_force_and_torque_arrows_from_their_frame(node, build, recording):
    build("wrench", entity_path="ft", params={"force_scale": 0.01, "torque_scale": 0.5})
    msg = WrenchStamped()
    msg.header.frame_id = "tool"
    msg.wrench.force.z, msg.wrench.torque.x = -20.0, 0.4
    node.publish(msg)

    row = recording.rows("/ft")[0]
    np.testing.assert_allclose(row["Arrows3D:origins"], [[0.0, 0.0, 0.0]] * 2)
    np.testing.assert_allclose(row["Arrows3D:vectors"], [[0.0, 0.0, -0.2], [0.2, 0.0, 0.0]], atol=1e-6)
    assert row["Arrows3D:labels"] == ["force", "torque"]
    assert row["CoordinateFrame:frame"] == ["tool"]


@pytest.mark.parametrize(
    ("module", "params"),
    [
        ("pose", {"axis_length": 0}),
        ("pose", {"n_sigma": -1}),
        ("pose_array", {"arrow_length": 0}),
        ("wrench", {"force_scale": 0}),
    ],
)
def test_lengths_and_scales_must_be_positive(build, module, params):
    with pytest.raises(ValueError, match="greater than 0"):
        build(module, params=params)
