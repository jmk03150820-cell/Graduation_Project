"""Pure-Python BEV-frame -> map-frame transform helpers. No ROS imports, so
testable without ROS/Autoware installed.

Assumes the RSU's BEV output is a local Cartesian frame (x=forward, y=left,
matching ROS REP-103 body-frame convention) centered on the RSU itself, and
that the RSU's own pose in the map frame (x, y, yaw) is known/configured.
This assumption is unconfirmed with the teammate — revisit once their real
publisher and its coordinate convention are known.
"""

import math


def bev_to_map_xy(pos_x, pos_y, rsu_x, rsu_y, rsu_yaw_deg):
    """Rotate+translate a BEV-frame point into the map frame."""
    yaw = math.radians(rsu_yaw_deg)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    map_x = rsu_x + pos_x * cos_yaw - pos_y * sin_yaw
    map_y = rsu_y + pos_x * sin_yaw + pos_y * cos_yaw
    return map_x, map_y


def bev_heading_to_map_yaw_rad(heading_deg, rsu_yaw_deg):
    """Adds the RSU's own map-frame yaw offset to a BEV-frame heading,
    normalized into [-pi, pi]."""
    yaw = math.radians(heading_deg + rsu_yaw_deg)
    return math.atan2(math.sin(yaw), math.cos(yaw))


def yaw_to_quaternion_xyzw(yaw_rad):
    half = yaw_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)
