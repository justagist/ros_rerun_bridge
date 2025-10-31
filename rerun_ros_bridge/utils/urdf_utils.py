# adapted from https://github.com/rerun-io/rerun/blob/main/examples/python/ros_node/rerun_urdf.py

from __future__ import annotations

import pathlib
from typing import Any, Dict

import ament_index_python
import numpy as np
import scipy.spatial.transform as st
import trimesh
from PIL import Image
from urdf_parser_py import urdf as urdf_parser

import rerun as rr


def _rpy_to_mat(rpy) -> np.ndarray:
    return st.Rotation.from_euler("xyz", rpy).as_matrix()


def _origin_to_mat(origin) -> np.ndarray:
    T = np.eye(4)
    if origin is not None and origin.xyz is not None:
        T[:3, 3] = origin.xyz
    if origin is not None and origin.rpy is not None:
        T[:3, :3] = _rpy_to_mat(origin.rpy)
    return T


def _axis_twist(axis, value, joint_type: str) -> np.ndarray:
    """Return a 4x4 transform for revolute/continuous/prismatic along 'axis' given value (rad or m)."""
    T = np.eye(4)
    axis = np.array(axis if axis is not None else [0, 0, 1], dtype=float)
    if joint_type in ("revolute", "continuous"):
        R = st.Rotation.from_rotvec(axis / (np.linalg.norm(axis) + 1e-12) * value).as_matrix()
        T[:3, :3] = R
    elif joint_type == "prismatic":
        T[:3, 3] = axis * value
    return T


class UrdfKinematics:
    """Simple URDF kinematics that logs per-joint Transform3D at the correct entity paths.

    We rely on Rerun's hierarchical composition: each joint's entity path corresponds to the
    child branch in the link tree, so composing all per-joint transforms yields the full pose.
    """

    def __init__(self, urdf: urdf_parser.URDF, root_path: str = "", link_scales: dict[str, float] | None = None) -> None:
        self.urdf = urdf
        self.root_path = root_path.rstrip("/")
        self.link_scales = link_scales or {}
        self.joint_info: Dict[str, Dict[str, Any]] = {}
        self._build_joint_index()

    def _root(self) -> str:
        return self.urdf.get_root()

    def _chain_entity_path_for_joint(self, joint: urdf_parser.Joint) -> str:
        # Path is the link chain from root to the joint's CHILD (skip joints in the string),
        # e.g. "base_link/…/child_link". We’ll log the joint’s Transform3D at this path.
        link_names = self.urdf.get_chain(self._root(), joint.child)[0::2]
        path = "/".join(link_names)
        return f"{self.root_path}/{path}" if self.root_path else path

    def _chain_entity_path_for_link(self, link: urdf_parser.Link) -> str:
        link_names = self.urdf.get_chain(self._root(), link.name)[0::2]
        path = "/".join(link_names)
        return f"{self.root_path}/{path}" if self.root_path else path

    def _build_joint_index(self) -> None:
        """Compute static origin transforms and entity paths for each joint."""
        for j in self.urdf.joints:
            ent = self._chain_entity_path_for_joint(j)
            self.joint_info[j.name] = {
                "entity_path": ent,
                "origin_T": _origin_to_mat(j.origin),
                "axis": (j.axis if j.axis is not None else [0, 0, 1]),
                "type": j.type,
                "limits": j.limit if hasattr(j, "limit") else None,
            }

    # ——— Public API used by JointState module ———
    def set_joint(self, name: str, value: float) -> None:
        info = self.joint_info.get(name)
        if not info:
            return
        T = info["origin_T"] @ _axis_twist(info["axis"], value, info["type"])
        rr.log(
            info["entity_path"],
            rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]),
        )

    def set_joints(self, names: list[str], values: list[float]) -> None:
        for n, v in zip(names, values):
            self.set_joint(n, float(v))

    # ——— One-time mesh logging (called by the URDF module when the model is received) ———
    def log_visuals(self) -> None:
        rr.log(self.root_path or "", rr.ViewCoordinates.RIGHT_HAND_Z_UP)
        for link in self.urdf.links:
            path = self._chain_entity_path_for_link(link)
            for i, visual in enumerate(link.visuals):
                self._log_visual(f"{path}/visual_{i}", visual, link_name=link.name)

    def log_visuals(self) -> None:
        rr.log(self.root_path or "", rr.ViewCoordinates.RIGHT_HAND_Z_UP)
        for link in self.urdf.links:
            path = self._chain_entity_path_for_link(link)
            for i, visual in enumerate(link.visuals):
                # pass the link name so we can look up scale
                self._log_visual(f"{path}/visual_{i}", visual, link_name=link.name)

    def _log_visual(self, entity_path: str, visual: urdf_parser.Visual, *, link_name: str) -> None:
        # Compose visual-local transform
        T = _origin_to_mat(visual.origin)

        # NEW: apply per-link uniform scale BEFORE geometry-specific transforms
        s_link = float(self.link_scales.get(link_name, 0.0) or 0.0)
        if s_link > 0.0:
            T[:3, :3] *= s_link

        # Geometry loading (mesh/box/cyl/sphere)
        if isinstance(visual.geometry, urdf_parser.Mesh):

            def resolve_ros_path(p: str) -> str:
                if p.startswith("package://"):
                    parts = pathlib.Path(p)
                    pkg = parts.parts[1]
                    rest = pathlib.Path(*parts.parts[2:])
                    base = ament_index_python.get_package_share_directory(pkg)
                    return str(pathlib.Path(base) / rest)
                if p.startswith("file://"):
                    return p[len("file://") :]
                return p

            mesh_path = resolve_ros_path(visual.geometry.filename)
            mesh_or_scene = trimesh.load_mesh(mesh_path)

            # If the mesh has its own <scale>, multiply it as well
            if visual.geometry.scale is not None:
                T[:3, :3] = T[:3, :3] * np.array(visual.geometry.scale)

        elif isinstance(visual.geometry, urdf_parser.Box):
            mesh_or_scene = trimesh.creation.box(extents=visual.geometry.size)
        elif isinstance(visual.geometry, urdf_parser.Cylinder):
            mesh_or_scene = trimesh.creation.cylinder(radius=visual.geometry.radius, height=visual.geometry.length)
        elif isinstance(visual.geometry, urdf_parser.Sphere):
            mesh_or_scene = trimesh.creation.icosphere(radius=visual.geometry.radius)
        else:
            rr.log(entity_path, rr.TextLog(f"Unsupported geometry: {type(visual.geometry)}"))
            mesh_or_scene = trimesh.Trimesh()

        mesh_or_scene.apply_transform(T)

        def _log_mesh(ep: str, mesh: trimesh.Trimesh) -> None:
            vertex_colors = None
            albedo_factor = None

            # Priority 1: embedded vertex colors from the mesh file
            if hasattr(mesh.visual, "vertex_colors") and len(mesh.visual.vertex_colors) > 0:
                vertex_colors = mesh.visual.vertex_colors

            # Priority 2: URDF <material> color (overrides)
            mat = visual.material
            if mat is not None:
                if mat.color is not None and mat.color.rgba is not None:
                    albedo_factor = np.array(mat.color.rgba)
                elif mat.texture is not None and mat.texture.filename:
                    tex_path = resolve_ros_path(mat.texture.filename)
                    tex_img = np.asarray(Image.open(tex_path))
                    rr.log(ep, rr.Image(tex_img))  # optional: log texture itself
                    rr.log(
                        ep,
                        rr.Mesh3D(
                            vertex_positions=mesh.vertices,
                            triangle_indices=mesh.faces,
                            vertex_normals=getattr(mesh, "vertex_normals", None),
                            albedo_texture=tex_img,
                            vertex_texcoords=getattr(mesh.visual, "uv", None),
                        ),
                    )
                    return  # done

            # Fallback: single uniform color if nothing else
            if vertex_colors is None and albedo_factor is None:
                albedo_factor = np.array([0.8, 0.8, 0.8, 1.0])

            rr.log(
                ep,
                rr.Mesh3D(
                    vertex_positions=mesh.vertices,
                    triangle_indices=mesh.faces,
                    vertex_normals=getattr(mesh, "vertex_normals", None),
                    vertex_colors=vertex_colors,
                    albedo_factor=albedo_factor,
                ),
            )

        if isinstance(mesh_or_scene, trimesh.Scene):
            for i, mesh in enumerate(mesh_or_scene.dump()):
                _log_mesh(f"{entity_path}/{i}", mesh)
        else:
            _log_mesh(entity_path, mesh_or_scene)
