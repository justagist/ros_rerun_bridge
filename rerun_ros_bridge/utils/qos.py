from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def make_qos(
    depth: int = 10,
    reliability: str | None = None,
    durability: str | None = None,
    history: str | None = None,
) -> QoSProfile:
    rel = {
        None: QoSReliabilityPolicy.RELIABLE,
        "reliable": QoSReliabilityPolicy.RELIABLE,
        "best_effort": QoSReliabilityPolicy.BEST_EFFORT,
    }[reliability]
    dur = {
        None: QoSDurabilityPolicy.VOLATILE,
        "volatile": QoSDurabilityPolicy.VOLATILE,
        "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL,
    }[durability]
    hist = {
        None: QoSHistoryPolicy.KEEP_LAST,
        "keep_last": QoSHistoryPolicy.KEEP_LAST,
        "keep_all": QoSHistoryPolicy.KEEP_ALL,
    }[history]
    return QoSProfile(depth=depth, reliability=rel, durability=dur, history=hist)
