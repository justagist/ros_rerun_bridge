"""
The main entry point for the ROS2 -> Rerun bridge node.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from urllib.parse import quote

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import rerun as rr

from . import __version__
from .base import BridgeConfig
from .builder import BridgeBuilder
from .context import BridgeContext


class BridgeNode(Node):
    """ROS2 -> Rerun bridge node."""

    def __init__(self, cfg: BridgeConfig) -> None:
        """Create the bridge node and its modules, and log the scene setup.

        Args:
            cfg: The validated config, from `load_config`.
        """
        super().__init__("ros_rerun_bridge")

        # Shared context. Currently only holds the robot model.
        bridge_context = BridgeContext()

        # The root entity's frame is the one Rerun's default 3D view renders in. ROS frames are right-handed, Z up.
        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, rr.CoordinateFrame(frame=cfg.global_frame), static=True)

        if cfg.blueprint:
            # Rerun applies it to this recording's application, whichever it was saved from
            rr.log_file_from_path(cfg.blueprint)

        # build modules
        builder = BridgeBuilder(self)
        self.modules = builder.build_modules(cfg, bridge_context)


def load_config(path: str | Path) -> BridgeConfig:
    """Load and validate a config, with its blueprint path resolved and checked.

    Args:
        path: Path to the YAML config.

    Returns:
        The config, with `blueprint` an absolute path if set.

    Raises:
        pydantic.ValidationError: If the config is invalid.
        FileNotFoundError: If the config or its blueprint does not exist.
    """
    with open(path) as f:
        cfg = BridgeConfig.model_validate(yaml.safe_load(f))
    if cfg.blueprint:
        blueprint = (Path(path).parent / cfg.blueprint).resolve()
        if not blueprint.is_file():
            raise FileNotFoundError(f"blueprint '{blueprint}' not found")
        cfg.blueprint = str(blueprint)
    return cfg


def serve(allowed_origins: list[str] | None, open_browser: bool) -> None:
    """Host a gRPC server and a web viewer, as `rr.script_setup` does for --serve, and print how to connect.

    Args:
        allowed_origins: Extra origins whose web viewers may connect to the gRPC server.
        open_browser: Open the web viewer in a browser on this machine.
    """
    rr.init(application_id="ros_rerun_bridge", default_enabled=True, strict=True)
    grpc_url = rr.get_global_data_recording().serve_grpc(cors_allow_origin=allowed_origins)
    rr.serve_web_viewer(open_browser=open_browser, connect_to=grpc_url)
    # The web viewer only connects when given the server in its `url` parameter
    print(
        "Connect Rerun viewers with (from another machine, replace 127.0.0.1/localhost with this machine's address):\n"
        f"  native:  rerun {grpc_url}   (add --port auto on this machine)\n"
        f"  browser: http://localhost:9090/?url={quote(grpc_url, safe='')}\n"
        "           (a browser reaching this machine by another address needs --allow-origin)",
        flush=True,
    )


def run(argv: list[str] | None = None) -> None:
    """Run the ROS2 -> Rerun bridge node.

    Args:
        argv: Command line arguments to use. If `None`, uses `sys.argv`.
    """
    argv = sys.argv[1:] if argv is None else argv

    # Let -v/--version bypass required args
    if "--version" in argv or "-v" in argv:
        print(f"ros-rerun-bridge version {__version__}")
        sys.exit(0)

    parser = argparse.ArgumentParser(description="ROS2 -> Rerun bridge (config + plugins)")

    # Group: app options
    bridge_grp = parser.add_argument_group()
    bridge_grp.add_argument("-v", "--version", action="store_true", help="Show version and exit")
    bridge_grp.add_argument("--config", required=True, help="Path to YAML config")
    bridge_grp.add_argument(
        "--allow-origin",
        action="append",
        metavar="PATTERN",
        help="With --serve: also let web viewers served from these origins connect, e.g. 'http://192.168.1.10:*' "
        "(by default only localhost and rerun.io). Repeatable.",
    )

    # Group: Rerun SDK options (show separately)
    rerun_grp = parser.add_argument_group("Rerun options (directly passed to Rerun SDK)")
    rr.script_add_args(rerun_grp)

    # Parse known so leftover go to rclpy, without the `--` separating them
    args, unknownargs = parser.parse_known_args(argv)
    unknownargs = [arg for arg in unknownargs if arg != "--"]
    # Before any viewer opens, so a config error is not hidden behind its window
    cfg = load_config(args.config)

    # Spawning the viewer looks `rerun` up on PATH. Use the one installed with this package, which matches the SDK
    # version, also when its environment is not on PATH (e.g. after `uv tool install`).
    os.environ["PATH"] = os.pathsep.join([os.path.dirname(sys.executable), os.environ.get("PATH", "")])
    if args.serve and not args.stdout:
        serve(args.allow_origin, open_browser=not args.headless)
    else:
        rr.script_setup(args, "ros_rerun_bridge")

    rclpy.init(args=unknownargs)
    node = BridgeNode(cfg)

    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy's signal handler shuts the context down from its own thread; shutting it down here as well races
        # with that and can fail with "rcl_shutdown already called"
        pass
    finally:
        node.destroy_node()
        # Flushes pending data, e.g. for --save
        rr.script_teardown(args)


if __name__ == "__main__":
    run()
