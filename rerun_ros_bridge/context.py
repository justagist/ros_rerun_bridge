from dataclasses import dataclass

from .tf_utils import TFHelper


@dataclass
class BridgeContext:
    tf: TFHelper
