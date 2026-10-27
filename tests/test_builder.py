import os
import subprocess
import sys

import pytest
from rclpy.qos import DurabilityPolicy
from visualization_msgs.msg import Marker

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


def test_resolves_a_module_by_python_path(build):
    assert isinstance(build("ros_rerun_bridge.modules.tf:TFModule"), TFModule)


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
