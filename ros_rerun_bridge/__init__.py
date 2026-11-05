import os
import sys
from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("ros-rerun-bridge")
except PackageNotFoundError:  # during dev
    __version__ = "0.0.0.dev"

_INSTALLATION = "https://github.com/justagist/ros_rerun_bridge/blob/main/docs/INSTALLATION.md"


def _check_environment() -> None:
    """Warn if the bridge runs in the system Python, and exit with what to do if it cannot import rclpy.

    Raises:
        SystemExit: If rclpy cannot be imported.
    """
    if sys.prefix == sys.base_prefix:
        print(
            "ros-rerun-bridge: warning: running in the system Python, not an environment of its own. The NumPy 2 it "
            f"installs can break other ROS nodes there, e.g. cv_bridge on Jazzy. See {_INSTALLATION}",
            file=sys.stderr,
        )
    try:
        import rclpy  # noqa: F401
    except ImportError as e:
        if "ROS_DISTRO" not in os.environ:
            reason = "ROS is not sourced: run e.g. `source /opt/ros/jazzy/setup.bash` first"
        else:
            # e.g. the bridge installed for a Python that uv downloaded, which rclpy is not compiled for
            python = f"{sys.version_info.major}.{sys.version_info.minor}"
            reason = (
                f"{e}. ROS {os.environ['ROS_DISTRO']}'s rclpy is compiled for its distro's Python, not Python "
                f"{python}: install the bridge with that Python"
            )
        sys.exit(f"ros-rerun-bridge: error: cannot import rclpy: {reason}. See {_INSTALLATION}")


def run(argv: list[str] | None = None) -> None:
    """Run the ROS2 -> Rerun bridge node, once the environment is checked (see `main.run`).

    Args:
        argv: Command line arguments to use. If `None`, uses `sys.argv`.
    """
    _check_environment()
    from .main import run as _run

    _run(argv)


__all__ = ["__version__", "run"]
