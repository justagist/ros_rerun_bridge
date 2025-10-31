# adapted from https://github.com/rerun-io/rerun/blob/main/examples/python/ros_node/rerun_urdf.py

from __future__ import annotations

import io
import os
from typing import TYPE_CHECKING, cast
from urllib.parse import urlparse

import numpy as np
from ament_index_python.packages import get_package_share_directory
from yourdfpy import URDF

import rerun as rr

if TYPE_CHECKING:
    import trimesh
    from std_msgs.msg import String


def ament_locate_package(fname: str) -> str:
    if not fname.startswith("package://"):
        return fname
    parsed = urlparse(fname)
    return os.path.join(get_package_share_directory(parsed.netloc), parsed.path[1:])


def load_urdf_from_msg(msg: "String") -> URDF:
    f = io.StringIO(msg.data)
    return URDF.load(f, filename_handler=ament_locate_package)


def log_scene(scene: "trimesh.Scene", node: str, path: str | None = None, static: bool = False) -> None:
    path = path + "/" + node if path else node

    parent = scene.graph.transforms.parents.get(node)
    children = scene.graph.transforms.children.get(node)

    node_data = scene.graph.get(frame_to=node, frame_from=parent)

    if node_data:
        if parent:
            world_from_mesh = node_data[0]
            rr.log(
                path,
                rr.Transform3D(
                    translation=world_from_mesh[3, 0:3],
                    mat3x3=world_from_mesh[0:3, 0:3],
                ),
                static=static,
            )
        mesh = cast("trimesh.Trimesh", scene.geometry.get(node_data[1]))
        if mesh:
            albedo_factor = None
            try:
                colors = np.mean(mesh.visual.vertex_colors, axis=0)
                if len(colors) == 4:
                    albedo_factor = np.array(colors) / 255.0
            except Exception:
                pass
            try:
                colors = mesh.visual.to_color().vertex_colors
                if len(colors) == 4:
                    albedo_factor = np.array(colors) / 255.0
            except Exception:
                pass
            rr.log(
                path,
                rr.Mesh3D(
                    vertex_positions=mesh.vertices,
                    triangle_indices=mesh.faces,
                    vertex_normals=mesh.vertex_normals,
                    albedo_factor=albedo_factor,
                ),
                static=static,
            )
    if children:
        for child in children:
            log_scene(scene, child, path, static)
