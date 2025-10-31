import numpy as np
from rclpy.time import Time

import rerun as rr


def set_rr_time_from_ros(stamp: Time) -> None:
    rr.set_time("ros_time", timestamp=np.datetime64(stamp.nanoseconds, "ns"))
