"""Pure-Python geometry helpers: lat/lon -> local ENU meters, heading -> yaw
-> quaternion. No ROS/Autoware imports, so this is testable without either
installed.

Uses the same flat-earth (equirectangular) approximation as
fake_rsu/message_generator.py, which is adequate for a single fake
intersection but NOT a substitute for Autoware's real map projector
(e.g. MGRS) once a real lanelet2 map is used. ``origin_lat``/``origin_lon``
here must match whatever map origin the rest of the stack assumes.
"""

import math

METERS_PER_DEG_LAT = 111320.0


def latlon_to_local_xy(lat_deg, lon_deg, origin_lat_deg, origin_lon_deg):
    """Returns (x, y) meters in a local ENU frame centered on the origin:
    x = east, y = north — matching ROS REP-103 (map frame, +x east, +y north)."""
    north_m = (lat_deg - origin_lat_deg) * METERS_PER_DEG_LAT
    east_m = ((lon_deg - origin_lon_deg) * METERS_PER_DEG_LAT
              * math.cos(math.radians(origin_lat_deg)))
    return east_m, north_m


def heading_deg_to_yaw_rad(heading_deg):
    """Converts a compass bearing (0=North, clockwise positive, degrees)
    into a ROS yaw (0=+x/East, counter-clockwise positive, radians)."""
    yaw = math.radians(90.0 - heading_deg)
    return math.atan2(math.sin(yaw), math.cos(yaw))  # normalize to [-pi, pi]


def yaw_to_quaternion_xyzw(yaw_rad):
    """Quaternion for a rotation of yaw_rad about the Z axis only."""
    half = yaw_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)
