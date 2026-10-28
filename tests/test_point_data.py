"""Point clouds, laser scans and particle clouds."""

import math

import numpy as np
import pytest
from geometry_msgs.msg import Pose
from nav2_msgs.msg import Particle, ParticleCloud
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

XYZ = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]


def cloud(points: list[tuple], fields: list[str], dtype: list[tuple], big_endian: bool = False) -> PointCloud2:
    raw = np.array(points, dtype=dtype)
    msg = PointCloud2(
        height=1,
        width=len(points),
        is_bigendian=big_endian,
        point_step=raw.itemsize,
        row_step=raw.nbytes,
        data=raw.tobytes(),
        is_dense=False,
    )
    # 4-byte fields are declared FLOAT32, as drivers declare packed `rgb` too; single bytes UINT8
    types = {4: PointField.FLOAT32, 1: PointField.UINT8}
    msg.fields = [
        PointField(name=name, offset=raw.dtype.fields[name][1], datatype=types[raw.dtype[name].itemsize], count=1)
        for name in fields
    ]
    msg.header.frame_id = "lidar"
    return msg


def test_packed_rgb_points_drop_only_nan_points_and_keep_colours_aligned(node, build, recording):
    build("pointcloud2", entity_path="points")
    xyz_rgb = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]
    # 0xFF8899AA: alpha 0xFF with red >= 0x80 makes the packed colour a NaN float, which must not drop the point
    node.publish(
        cloud(
            [(1.0, 2.0, 3.0, 0xFF8899AA), (math.nan, 0.0, 0.0, 0x00010203), (4.0, 5.0, 6.0, 0x00112233)],
            ["x", "y", "z", "rgb"],
            xyz_rgb,
        )
    )

    row = recording.rows("/points")[0]
    assert row["Points3D:positions"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    # Rerun packs colours as 0xRRGGBBAA
    assert row["Points3D:colors"] == [0x8899AAFF, 0x112233FF]
    assert row["CoordinateFrame:frame"] == ["lidar"]


def test_colour_fields_mode_reads_separate_channels(node, build, recording):
    build("pointcloud2", entity_path="points", params={"colour_mode": "fields"})
    dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")]
    node.publish(cloud([(1.0, 1.0, 1.0, 255, 128, 0)], ["x", "y", "z", "r", "g", "b"], dtype))
    assert recording.rows("/points")[0]["Points3D:colors"] == [0xFF8000FF]


def test_clouds_without_colour_get_no_colour_by_default(node, build, recording):
    build("pointcloud2", entity_path="points")
    node.publish(cloud([(1.0, 2.0, 3.0)], ["x", "y", "z"], XYZ))
    row = recording.rows("/points")[0]
    assert row["Points3D:positions"] == [[1.0, 2.0, 3.0]]
    # Empty, so Rerun's default colour is used
    assert row["Points3D:colors"] == []


def test_a_cloud_without_colour_does_not_keep_an_earlier_clouds_colours(node, build, recording):
    build("pointcloud2", entity_path="points")
    xyz_rgb = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]
    node.publish(cloud([(1.0, 1.0, 1.0, 0x00FF0000)], ["x", "y", "z", "rgb"], xyz_rgb))
    node.publish(cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)], ["x", "y", "z"], XYZ))
    assert [row["Points3D:colors"] for row in recording.rows("/points")] == [[0xFF0000FF], []]


def test_organised_clouds_with_row_padding_keep_every_point(node, build, recording):
    build("pointcloud2", entity_path="points")
    xyz_rgb = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]
    msg = cloud([(1.0, 2.0, 3.0, 0x00FF0000), (4.0, 5.0, 6.0, 0x000000FF)], ["x", "y", "z", "rgb"], xyz_rgb)
    # One point per row, each row followed by 8 bytes of padding, which must not be read as a point
    rows = np.frombuffer(msg.data, np.uint8).reshape(2, 16)
    msg.height, msg.width, msg.row_step = 2, 1, 24
    msg.data = np.hstack([rows, np.full((2, 8), 0x7F, np.uint8)]).tobytes()
    node.publish(msg)
    row = recording.rows("/points")[0]
    assert row["Points3D:positions"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert row["Points3D:colors"] == [0xFF0000FF, 0x0000FFFF]


def test_big_endian_clouds_are_read_in_their_byte_order(node, build, recording):
    build("pointcloud2", entity_path="points")
    dtype = [("x", ">f4"), ("y", ">f4"), ("z", ">f4"), ("rgb", ">u4")]
    node.publish(cloud([(1.0, 2.0, 3.0, 0x00112233)], ["x", "y", "z", "rgb"], dtype, big_endian=True))
    row = recording.rows("/points")[0]
    assert row["Points3D:positions"] == [[1.0, 2.0, 3.0]]
    assert row["Points3D:colors"] == [0x112233FF]


def test_no_colour_mode_ignores_a_colour_field(node, build, recording):
    build("pointcloud2", entity_path="points", params={"colour_mode": "none"})
    xyz_rgb = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]
    node.publish(cloud([(1.0, 1.0, 1.0, 0x00112233)], ["x", "y", "z", "rgb"], xyz_rgb))
    row = recording.rows("/points")[0]
    assert row["Points3D:positions"] == [[1.0, 1.0, 1.0]]
    assert row["Points3D:colors"] == []


def scan() -> LaserScan:
    msg = LaserScan(angle_min=-0.1, angle_max=0.1, angle_increment=0.1, range_min=0.1, range_max=10.0)
    msg.ranges = [1.0, 2.0, 3.0]
    msg.header.frame_id = "base_scan"
    return msg


def test_laser_scan_lines_draw_one_segment_per_ray(node, build, recording):
    build("laserscan", entity_path="scan", params={"origin_scale": 0.3})
    node.publish(scan())

    row = recording.rows("/scan")[0]
    strips = np.array(row["LineStrips3D:strips"])
    assert strips.shape == (3, 2, 3)
    np.testing.assert_allclose(np.linalg.norm(strips[:, 0], axis=1), 0.3, rtol=1e-5)
    np.testing.assert_allclose(np.linalg.norm(strips[:, 1], axis=1), [1.0, 2.0, 3.0], rtol=1e-5)
    assert row["CoordinateFrame:frame"] == ["base_scan"]


def test_laser_scan_rays_start_at_returns_nearer_than_origin_scale(node, build, recording):
    build("laserscan", entity_path="scan", params={"origin_scale": 0.3})
    msg = scan()
    msg.ranges = [0.2, 2.0, 0.15]
    node.publish(msg)
    strips = np.array(recording.rows("/scan")[0]["LineStrips3D:strips"])
    np.testing.assert_allclose(np.linalg.norm(strips[:, 0], axis=1), [0.2, 0.3, 0.15], rtol=1e-5)


def test_laser_scan_without_returns_logs_no_rays(node, build, recording):
    build("laserscan", entity_path="scan")
    msg = scan()
    msg.ranges = [float("inf"), 20.0, 0.0]
    node.publish(msg)
    assert recording.rows("/scan")[0]["LineStrips3D:strips"] == []
    assert not [m for m in node.log_messages if m[0] == "error"]


def test_laser_scan_points_mode(node, build, recording):
    build("laserscan", entity_path="scan", params={"mode": "points"})
    node.publish(scan())
    positions = np.array(recording.rows("/scan")[0]["Points3D:positions"])
    angles = [-0.1, 0.0, 0.1]
    expected = [[r * np.cos(a), r * np.sin(a), 0.0] for r, a in zip([1.0, 2.0, 3.0], angles)]
    np.testing.assert_allclose(positions, expected, rtol=1e-5, atol=1e-6)


def particle_cloud():
    particles = [Particle(pose=Pose(), weight=w) for w in (0.2, 1.0)]
    for i, p in enumerate(particles):
        p.pose.position.x, p.pose.orientation.w = float(i), 1.0
    msg = ParticleCloud(particles=particles)
    msg.header.frame_id = "map"
    return msg


def test_particles_as_points_fade_with_weight(node, build, recording):
    build("particle_cloud", entity_path="particles")
    node.publish(particle_cloud())
    row = recording.rows("/particles")[0]
    assert row["Points3D:positions"] == [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
    # The lightest particle keeps 15% alpha, the heaviest is opaque
    assert row["Points3D:colors"] == [0xFFFFFF26, 0xFFFFFFFF]


def test_particle_arrows_scale_with_weight(node, build, recording):
    build("particle_cloud", entity_path="particles", params={"mode": "arrows", "arrow_len": 0.1})
    node.publish(particle_cloud())
    # Weights normalised over the message (0.2 to 1.0), lengths from 20% to 100% of arrow_len
    vectors = np.array(recording.rows("/particles")[0]["Arrows3D:vectors"])
    np.testing.assert_allclose(vectors[:, 0], [0.02, 0.1], rtol=1e-5)


def test_particle_weights_can_be_normalised_over_a_fixed_range(node, build, recording):
    params = {"mode": "arrows", "arrow_len": 0.1, "w_min": 0.0, "w_max": 2.0}
    build("particle_cloud", entity_path="particles", params=params)
    node.publish(particle_cloud())
    vectors = np.array(recording.rows("/particles")[0]["Arrows3D:vectors"])
    np.testing.assert_allclose(vectors[:, 0], [0.1 * (0.2 + 0.8 * 0.1), 0.1 * (0.2 + 0.8 * 0.5)], rtol=1e-5)


def test_particles_keep_their_colour_without_colour_by_weight(node, build, recording):
    build("particle_cloud", entity_path="particles", params={"colour_by_weight": False, "colour": [0, 200, 255, 128]})
    node.publish(particle_cloud())
    assert recording.rows("/particles")[0]["Points3D:colors"] == [0x00C8FF80, 0x00C8FF80]


@pytest.mark.parametrize(("arrow_axis", "axis"), [("y", 1), ("z", 2)])
def test_particle_arrows_point_along_the_chosen_body_axis(node, build, recording, arrow_axis, axis):
    build("particle_cloud", entity_path="particles", params={"mode": "arrows", "arrow_axis": arrow_axis})
    node.publish(particle_cloud())
    vectors = np.array(recording.rows("/particles")[0]["Arrows3D:vectors"])
    # The particles are unrotated: every arrow is along that world axis
    assert np.count_nonzero(vectors, axis=0).tolist() == [2 if i == axis else 0 for i in range(3)]


@pytest.mark.parametrize(("mode", "shape"), [("points", "Points3D:positions"), ("arrows", "Arrows3D:vectors")])
def test_an_empty_particle_cloud_clears_the_particles(node, build, recording, mode, shape):
    build("particle_cloud", entity_path="particles", params={"mode": mode})
    node.publish(particle_cloud())
    node.publish(ParticleCloud())
    assert [len(row[shape]) for row in recording.rows("/particles")] == [2, 0]


def test_particles_fade_from_the_colours_own_alpha(node, build, recording):
    build("particle_cloud", entity_path="particles", params={"colour": [255, 255, 255, 100]})
    node.publish(particle_cloud())
    # 15% of the alpha for the lightest particle, all of it for the heaviest
    assert recording.rows("/particles")[0]["Points3D:colors"] == [0xFFFFFF0F, 0xFFFFFF64]
