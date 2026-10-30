"""Robot model and joint states."""

import math
import re
from pathlib import Path

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ros_rerun_bridge.utils.meshes import collada_unit

DATA_DIR = Path(__file__).parent / "data"

URDF = f"""<robot name="tester">
  <link name="base">
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="arm">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <visual><geometry><mesh filename="file://{DATA_DIR / "triangle_inches.dae"}" scale="2 2 2"/></geometry></visual>
  </link>
  <link name="finger"/>
  <link name="tip"/>
  <!-- A fixed link, which joint states never carry -->
  <link name="camera"/>
  <joint name="camera_joint" type="fixed">
    <parent link="arm"/><child link="camera"/>
    <origin xyz="0 0 0.2"/>
  </joint>
  <!-- Mimics a mimic joint, and comes first so a single pass would miss it -->
  <joint name="tip_joint" type="revolute">
    <parent link="finger"/><child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
    <mimic joint="finger_joint" multiplier="2"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="finger_joint" type="revolute">
    <parent link="arm"/><child link="finger"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
    <mimic joint="shoulder" multiplier="-0.5" offset="0.1"/>
  </joint>
</robot>"""

BASE = "/robot/tester/visual_geometries/base/visual_0"
ARM_BOX = "/robot/tester/visual_geometries/arm/visual_0"
ARM = "/robot/tester/visual_geometries/arm/visual_1"


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
    assert sorted(row["Transform3D:child_frame"][0] for row in rows) == ["arm", "camera", "finger", "tip"]


def test_collada_meshes_are_scaled_by_their_unit_and_urdf_scale(robot, recording):
    # The mesh is in inches (0.0254 m) and the URDF scales it by 2; the last scale logged wins
    assert recording.latest(ARM, "InstancePoses3D:scales") == [pytest.approx([0.0508] * 3)]


def test_other_geometry_is_not_rescaled(robot, recording):
    # Including another visual on the same link as the mesh
    assert recording.latest(BASE, "InstancePoses3D:scales") is None
    assert recording.latest(ARM_BOX, "InstancePoses3D:scales") is None


@pytest.mark.parametrize(
    "filename",
    ["package://not_a_package/meshes/x.dae", "file:///does/not/exist.dae", "/does/not/exist.dae"],
)
def test_collada_meshes_that_cannot_be_found_are_not_rescaled(filename):
    assert collada_unit(filename) == 1.0


@pytest.mark.parametrize("declaration", ["<unit meter='0.0254'/>", '<unit meter = "0.0254" name="inch"/>'])
def test_collada_units_are_read_with_either_quotes_and_spacing(tmp_path, declaration):
    mesh = tmp_path / "part.dae"
    mesh.write_text(f"<COLLADA><asset>{declaration}</asset></COLLADA>")
    assert collada_unit(str(mesh)) == pytest.approx(0.0254)


def test_collada_units_are_read_from_package_paths(tmp_path, monkeypatch):
    share = tmp_path / "share" / "test_meshes"
    (share / "meshes").mkdir(parents=True)
    (share / "meshes" / "part.dae").write_bytes((DATA_DIR / "triangle_inches.dae").read_bytes())
    (tmp_path / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    (tmp_path / "share" / "ament_index" / "resource_index" / "packages" / "test_meshes").write_text("")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path))
    assert collada_unit("package://test_meshes/meshes/part.dae") == pytest.approx(0.0254)


def z_rotation(angle: float) -> list:
    return [pytest.approx([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])]


def joints_by_child(recording) -> dict:
    return {row["Transform3D:child_frame"][0]: row for row in recording.rows("/tf")}


def test_joint_states_move_joints_beyond_their_limits(robot, node, build, recording):
    build("joint_states", entity_path="tf")
    msg = JointState(name=["shoulder", "not_a_joint"], position=[2.0, 1.0])
    msg.header.stamp = TimeMsg(sec=3)
    node.publish(msg)

    arm = joints_by_child(recording)["arm"]
    assert arm["ros_time"] == 3_000_000_000
    assert arm["Transform3D:parent_frame"] == ["base"]
    # 2 rad about z, not clamped to the 1 rad limit
    assert arm["Transform3D:quaternion"] == z_rotation(2.0)


def test_mimic_joints_follow_the_joint_they_mimic_including_chains(robot, node, build, recording):
    build("joint_states", entity_path="tf")
    node.publish(JointState(name=["shoulder"], position=[1.0]))
    joints = joints_by_child(recording)
    assert joints["finger"]["Transform3D:quaternion"] == z_rotation(-0.5 * 1.0 + 0.1)
    assert joints["tip"]["Transform3D:quaternion"] == z_rotation(2 * (-0.5 * 1.0 + 0.1))


def test_mimic_joints_given_in_the_message_keep_their_own_position(robot, node, build, recording):
    build("joint_states", entity_path="tf")
    node.publish(JointState(name=["shoulder", "finger_joint"], position=[1.0, 0.7]))
    assert joints_by_child(recording)["finger"]["Transform3D:quaternion"] == z_rotation(0.7)


def test_joint_states_wait_for_the_robot_model(node, build, recording):
    build("joint_states", entity_path="tf")
    node.publish(JointState(name=["shoulder"], position=[0.5]))
    assert "/tf" not in recording.entities()
    assert node.log_messages[-1][0] == "warn"


def test_joint_states_draw_each_link_frame_on_its_own_entity(robot, node, build, recording):
    build("joint_states", entity_path="tf", params={"axis_length": 0.1, "frame_names": True})
    node.publish(JointState(name=["shoulder"], position=[0.5]))
    # Every link, including the fixed camera link and links not in the message
    frames = sorted(entity for entity in recording.entities() if entity.startswith("/frames/"))
    assert frames == ["/frames/arm", "/frames/base", "/frames/camera", "/frames/finger", "/frames/tip"]
    axes = recording.rows("/frames/arm")[0]
    assert axes["TransformAxes3D:axis_length"] == pytest.approx([0.1])
    assert axes["TransformAxes3D:show_frame"] == [True]


def test_joint_states_draw_the_frames_of_a_new_robot_model(robot, node, build, recording):
    build("joint_states", entity_path="tf", params={"axis_length": 0.1})
    node.publish(JointState(name=["shoulder"], position=[0.5]))
    gripper = (
        '<link name="gripper"/>'
        '<joint name="gripper_joint" type="fixed"><parent link="tip"/><child link="gripper"/></joint>'
    )
    node.publish(String(data=URDF.replace("</robot>", gripper + "</robot>")), index=0)
    node.publish(JointState(name=["shoulder"], position=[0.5]))
    assert "/frames/gripper" in recording.entities()


def arrival(recording, entity: str, column: str) -> list:
    """The values of a column logged on an entity, in the order the recording received them."""
    values = []
    for chunk in recording.chunks():
        batch = chunk.to_record_batch()
        if chunk.entity_path == entity and column in batch.schema.names:
            values += batch.column(column).to_pylist()
    return values


def test_a_new_robot_model_clears_the_old_models_geometry_but_not_its_frames(robot, node, recording):
    # E.g. a link's box replaced by a sphere, and the arm's mesh removed: they would otherwise stay
    urdf = URDF.replace('<box size="0.2 0.2 0.2"/>', '<sphere radius="0.1"/>', 1)
    node.publish(String(data=re.sub(r"\s*<visual><geometry><mesh [^>]*/></geometry></visual>", "", urdf)), index=0)
    assert arrival(recording, BASE, "Boxes3D:half_sizes")[-1] == []
    assert arrival(recording, ARM, "Asset3D:blob")[-1] == []
    # Kept, so the removed mesh's entity still has a path to the scene's frames
    assert [] not in arrival(recording, ARM, "CoordinateFrame:frame")


def test_a_new_robot_model_arrives_after_the_old_one_is_cleared(robot, node, recording):
    # The same model again: Rerun keeps the static data it received last, which must be the new model's
    node.publish(String(data=URDF), index=0)
    assert arrival(recording, ARM_BOX, "Boxes3D:half_sizes")[-1] == [pytest.approx([0.05, 0.05, 0.05])]


def test_a_robot_model_that_fails_to_load_leaves_the_old_one(robot, node, context, recording):
    node.publish(String(data="<robot"), index=0)
    assert node.log_messages[-2][0] == "error"
    assert [] not in arrival(recording, BASE, "Boxes3D:half_sizes")
    assert context.urdf_tree is robot


def test_a_new_robot_model_clears_the_old_one_under_an_escaped_entity_path(node, build, recording):
    # The importer escapes the prefix's slash and space as one entity path part
    build("robot_description", entity_path="scene/my robot")
    node.publish(String(data=URDF))
    node.publish(String(data=URDF.replace('<box size="0.2 0.2 0.2"/>', '<sphere radius="0.1"/>', 1)))
    base = BASE.replace("/robot/", "/scene\\/my\\ robot/")
    assert arrival(recording, base, "Boxes3D:half_sizes")[-1] == []
