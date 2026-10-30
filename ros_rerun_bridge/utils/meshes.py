"""Mesh files that ROS messages refer to, for the modules that load them: `robot_description` and `marker_array`."""

from __future__ import annotations

import re
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

_COLLADA_UNIT = re.compile(rb"""<unit\b[^>]*\bmeter\s*=\s*(?:"([^"]+)"|'([^']+)')""")


def mesh_path(filename: str) -> Path | None:
    """Return the file a mesh path names, or None if it names a package that cannot be found.

    Args:
        filename: Mesh path: `package://`, `file://` or absolute.
    """
    if filename.startswith("package://"):
        package, _, relative = filename[len("package://") :].partition("/")
        try:
            return Path(get_package_share_directory(package)) / relative
        except PackageNotFoundError:
            return None
    return Path(filename.removeprefix("file://"))


def collada_unit(filename: str) -> float:
    """Return the metres per unit a Collada file declares, or 1.0 if it declares none or cannot be found.

    Args:
        filename: Mesh path: `package://`, `file://` or absolute.
    """
    path = mesh_path(filename)
    if path is None or not path.is_file():
        return 1.0
    # <unit> is in the <asset> block at the top of the file
    with path.open("rb") as f:
        match = _COLLADA_UNIT.search(f.read(16384))
    return float(match.group(1) or match.group(2)) if match else 1.0
