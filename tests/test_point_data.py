"""Point clouds, laser scans and particle clouds."""

import math

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def cloud(points: list[tuple], fields: list[str], dtype: list[tuple]) -> PointCloud2:
    raw = np.array(points, dtype=dtype)
    msg = PointCloud2(
        height=1,
        width=len(points),
        is_bigendian=False,
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


def test_no_colour_mode_logs_positions_only(node, build, recording):
    build("pointcloud2", entity_path="points", params={"colour_mode": "none"})
    node.publish(cloud([(1.0, 1.0, 1.0)], ["x", "y", "z"], [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]))
    row = recording.rows("/points")[0]
    assert row["Points3D:positions"] == [[1.0, 1.0, 1.0]]
    assert row.get("Points3D:colors") is None


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


def test_laser_scan_points_mode(node, build, recording):
    build("laserscan", entity_path="scan", params={"mode": "points"})
    node.publish(scan())
    assert len(recording.rows("/scan")[0]["Points3D:positions"]) == 3


def test_particle_clouds_as_points_and_arrows(node, build, recording):
    nav2_msgs = pytest.importorskip("nav2_msgs.msg")
    from geometry_msgs.msg import Pose

    particles = [nav2_msgs.Particle(pose=Pose(), weight=w) for w in (0.2, 1.0)]
    for p in particles:
        p.pose.orientation.w = 1.0
    msg = nav2_msgs.ParticleCloud(particles=particles)
    msg.header.frame_id = "map"

    build("particle_cloud", entity_path="particles/points")
    node.publish(msg)
    build("particle_cloud", entity_path="particles/arrows", params={"mode": "arrows", "arrow_len": 0.1})
    node.publish(msg)

    assert len(recording.rows("/particles/points")[0]["Points3D:positions"]) == 2
    # Arrow length scales with weight, with a floor
    vectors = np.array(recording.rows("/particles/arrows")[0]["Arrows3D:vectors"])
    np.testing.assert_allclose(vectors[:, 0], [0.1 * 0.2, 0.1], rtol=1e-5)
