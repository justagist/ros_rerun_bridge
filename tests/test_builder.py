import os
import subprocess
import sys

import pytest
from pydantic import ValidationError
from rclpy.qos import DurabilityPolicy
from visualization_msgs.msg import Marker

from ros_rerun_bridge.main import load_config
from ros_rerun_bridge.modules.tf import TFModule


def test_only_imports_the_built_in_modules_a_config_uses():
    # A fresh interpreter: other tests import every module
    code = (
        "import sys\n"
        "from tests.conftest import FakeNode\n"
        "from ros_rerun_bridge.base import BridgeConfig\n"
        "from ros_rerun_bridge.builder import BridgeBuilder\n"
        "from ros_rerun_bridge.context import BridgeContext\n"
        "config = BridgeConfig(modules=[{'name': 'tf', 'module': 'tf', 'topic': '/tf', 'entity_path': 'tf'}])\n"
        "BridgeBuilder(FakeNode()).build_modules(config, BridgeContext())\n"
        "print(sorted(m for m in sys.modules if m.startswith('ros_rerun_bridge.modules.')))\n"
    )
    root = os.path.dirname(os.path.dirname(__file__))
    result = subprocess.run([sys.executable, "-c", code], cwd=root, capture_output=True, text=True, check=True)
    assert result.stdout.strip() == "['ros_rerun_bridge.modules.tf']"


def test_joint_states_does_not_import_the_tf_module():
    # It shares the frame axes code with tf, which must not pull in tf2_msgs or register tf
    code = (
        "import sys\n"
        "import ros_rerun_bridge.modules.joint_states\n"
        "print('ros_rerun_bridge.modules.tf' in sys.modules, 'tf2_msgs.msg' in sys.modules)\n"
    )
    root = os.path.dirname(os.path.dirname(__file__))
    result = subprocess.run([sys.executable, "-c", code], cwd=root, capture_output=True, text=True, check=True)
    assert result.stdout.strip() == "False False"


def test_resolves_a_module_by_python_path(build):
    assert isinstance(build("ros_rerun_bridge.modules.tf:TFModule"), TFModule)


@pytest.mark.parametrize("path", ["ros_rerun_bridge.utils.utils:make_qos", "rclpy.node:Node"])
def test_module_path_to_anything_but_a_module_class_is_an_error(build, path):
    with pytest.raises(ValueError, match="is not a TopicToComponentModule subclass"):
        build(path)


def test_unknown_module_name_is_an_error(build):
    with pytest.raises(ValueError, match="built-in module"):
        build("tff")


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


@pytest.mark.parametrize(
    ("qos", "error"), [({"reliability": "best-effort"}, "reliability"), ({"depth": 0}, "depth"), ({"deph": 5}, "deph")]
)
def test_invalid_qos_is_an_error(build, qos, error):
    with pytest.raises(ValidationError, match=error):
        build("tf", qos=qos)


@pytest.mark.parametrize(
    ("module", "params"),
    [
        ("range", {"colour": [0, 300, 0, 255]}),
        ("particle_cloud", {"colour": [-1, 0, 0, 255]}),
        ("scalars", {"fields": ["range"], "colours": [[0, 0, 256, 255]]}),
        ("bar_chart", {"field": "effort", "colour": [0, 0, 0, 300]}),
    ],
)
def test_colour_channels_outside_0_to_255_are_an_error(build, module, params):
    # Rerun would wrap them round instead of rejecting them
    msg_type = {"scalars": "sensor_msgs.msg.Range", "bar_chart": "sensor_msgs.msg.JointState"}.get(module)
    with pytest.raises(ValueError, match="(less|greater) than or equal to"):
        build(module, msg_type=msg_type, params=params)


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
    with pytest.raises(ValidationError, match="modules"):
        load_config(write_config(tmp_path, "global_frame: map\n"))


def test_version_flag_prints_the_version_without_a_config():
    result = subprocess.run(
        [sys.executable, "-m", "ros_rerun_bridge.main", "--version"], capture_output=True, text=True
    )
    assert result.returncode == 0 and result.stdout.startswith("ros-rerun-bridge version ")
