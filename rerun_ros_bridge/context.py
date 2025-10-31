from dataclasses import dataclass

from .utils.tf_utils import TFHelper


@dataclass
class BridgeContext:
    tf: TFHelper
