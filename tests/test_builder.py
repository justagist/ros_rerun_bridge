import os
import subprocess
import sys

import pytest
from pydantic import ValidationError
from rclpy.qos import DurabilityPolicy
from visualization_msgs.msg import Marker

from ros_rerun_bridge import main
from ros_rerun_bridge.main import load_config
from ros_rerun_bridge.modules.tf import TFModule


# joint_states shares the frame axes code with tf, which must not pull tf in
@pytest.mark.parametrize("module", ["tf", "joint_states"])
def test_only_imports_the_built_in_modules_a_config_uses(module):
    # A fresh interpreter: other tests import every module
    code = (
        "import sys\n"
        "from tests.conftest import FakeNode\n"
        "from ros_rerun_bridge.base import BridgeConfig\n"
        "from ros_rerun_bridge.builder import BridgeBuilder\n"
        "from ros_rerun_bridge.context import BridgeContext\n"
        f"config = BridgeConfig(modules=[{{'name': 'm', 'module': '{module}', 'topic': '/t', 'entity_path': 'e'}}])\n"
        "BridgeBuilder(FakeNode()).build_modules(config, BridgeContext())\n"
        "print(sorted(m for m in sys.modules if m.startswith('ros_rerun_bridge.modules.')))\n"
    )
    root = os.path.dirname(os.path.dirname(__file__))
    result = subprocess.run([sys.executable, "-c", code], cwd=root, capture_output=True, text=True, check=True)
    assert result.stdout.strip() == f"['ros_rerun_bridge.modules.{module}']"


def test_resolves_a_module_by_python_path(build):
    assert isinstance(build("ros_rerun_bridge.modules.tf:TFModule"), TFModule)


@pytest.mark.parametrize("path", ["ros_rerun_bridge.utils.utils:make_qos", "rclpy.node:Node"])
def test_module_path_to_anything_but_a_module_class_is_an_error(build, path):
    with pytest.raises(ValueError, match="is not a TopicToComponentModule subclass"):
        build(path)


def test_unknown_module_name_is_an_error(build):
    with pytest.raises(ValueError, match="built-in module"):
        build("tff")


def test_a_config_error_names_the_module(build):
    with pytest.raises(ValueError, match=r"^\[scalars\] field 'distance'"):
        build("scalars", msg_type="sensor_msgs.msg.Range", params={"fields": ["distance"]})


def test_params_for_a_module_without_any_are_an_error(build):
    with pytest.raises(ValueError, match="invalid 'params'"):
        build("image", params={"colormap": "Viridis"})


def test_unknown_params_are_an_error(build):
    with pytest.raises(ValueError, match="invalid 'params'"):
        build("tf", params={"statik": True})


def test_subscribes_with_the_configured_topic_qos_and_message_type(node, build):
    build(
        "marker_array", topic="/marker", msg_type="visualization_msgs.msg.Marker", qos={"durability": "transient_local"}
    )
    subscription = node.subscriptions[-1]
    assert subscription.topic == "/marker"
    assert subscription.msg_type is Marker
    assert subscription.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


class SubscriptionsSeenInInit(TFModule):
    """Records the subscriptions that exist once its own initialisation is done."""

    def __init__(self, node, config, context) -> None:
        super().__init__(node, config, context)
        self.subscriptions_in_init = list(node.subscriptions)


def test_subscribes_only_once_the_module_is_built(node, build):
    # With automatic discovery, modules are built while the executor runs: a message must not reach a half-built one
    module = build(f"{__name__}:SubscriptionsSeenInInit")
    assert module.subscriptions_in_init == []
    assert len(node.subscriptions) == 1


def test_misspelt_qos_settings_are_an_error(build):
    with pytest.raises(ValidationError, match="deph"):
        build("tf", qos={"deph": 5})


def test_colour_channels_outside_0_to_255_are_an_error(build):
    # Rerun would wrap them round instead of rejecting them
    with pytest.raises(ValueError, match="less than or equal to"):
        build("range", params={"colour": [0, 300, 0, 255]})


def write_config(directory, text: str):
    path = directory / "bridge.yaml"
    path.write_text(text)
    return path


def test_config_blueprint_is_resolved_relative_to_the_config(tmp_path):
    (tmp_path / "layout.rbl").write_bytes(b"")
    cfg = load_config(write_config(tmp_path, "blueprint: layout.rbl\nmodules: []\n"))
    assert cfg.blueprint == str(tmp_path / "layout.rbl")


def test_config_with_a_missing_blueprint_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError, match="missing.rbl"):
        load_config(write_config(tmp_path, "blueprint: missing.rbl\nmodules: []\n"))


def test_invalid_config_is_an_error(tmp_path):
    with pytest.raises(ValidationError, match="globl_frame"):
        load_config(write_config(tmp_path, "globl_frame: map\n"))


def run_bridge(*args: str) -> subprocess.CompletedProcess:
    """Run the bridge's command line in a fresh interpreter."""
    return subprocess.run([sys.executable, "-m", "ros_rerun_bridge.main", *args], capture_output=True, text=True)


def test_without_a_config_or_auto_is_an_error():
    result = run_bridge()
    assert result.returncode == 2 and "--config is required, unless --auto is given" in result.stderr


def test_a_config_without_modules_needs_auto(tmp_path):
    result = run_bridge("--config", str(write_config(tmp_path, "global_frame: world\n")))
    assert result.returncode == 2 and "the config lists no modules" in result.stderr


def test_version_flag_prints_the_version_without_a_config():
    result = run_bridge("--version")
    assert result.returncode == 0 and result.stdout.startswith("ros-rerun-bridge version ")


class Started(Exception):
    """Stops the command line where it starts the bridge node, carrying the config the node was given."""


@pytest.fixture
def start(monkeypatch):
    """Run the command line up to starting the bridge node, without ROS or a viewer; returns the node's config."""
    monkeypatch.setattr(main.rr, "init", lambda **kwargs: None)
    monkeypatch.setattr(main.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(main.rclpy, "try_shutdown", lambda: None)

    def _start(*argv: str, node=None):
        def bridge_node(cfg):
            raise Started(cfg)

        monkeypatch.setattr(main, "BridgeNode", node or bridge_node)
        with pytest.raises(Started) as started:
            main.run(list(argv))
        return started.value.args[0]

    return _start


def test_auto_keeps_the_configs_auto_settings(start, tmp_path):
    cfg = start("--auto", "--config", str(write_config(tmp_path, "auto:\n  exclude: ['/rosout']\n")))
    assert cfg.auto.exclude == ["/rosout"]


def test_global_frame_overrides_the_configs(start, tmp_path):
    cfg = start("--global-frame", "world", "--config", str(write_config(tmp_path, "global_frame: map\nauto: true\n")))
    assert cfg.global_frame == "world"


def test_a_module_that_cannot_be_built_stops_the_bridge(start, tmp_path, capsys):
    def failing_node(cfg):
        raise ValueError("[scan] invalid 'params'")

    with pytest.raises(SystemExit) as exited:
        start("--auto", node=failing_node)
    assert exited.value.code == 2 and "[scan] invalid 'params'" in capsys.readouterr().err
