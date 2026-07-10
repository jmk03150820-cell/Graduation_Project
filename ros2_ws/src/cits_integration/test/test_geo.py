"""Pure-Python tests for geo.py; no ROS/Autoware install required.

Run standalone with:  python3 -m pytest test/test_geo.py
"""

import math

from cits_integration.geo import (
    heading_deg_to_yaw_rad,
    latlon_to_local_xy,
    yaw_to_quaternion_xyzw,
)


def test_origin_maps_to_zero():
    x, y = latlon_to_local_xy(37.5665, 126.9780, 37.5665, 126.9780)
    assert abs(x) < 1e-9
    assert abs(y) < 1e-9


def test_north_offset_increases_y_only():
    x, y = latlon_to_local_xy(37.5675, 126.9780, 37.5665, 126.9780)
    assert abs(x) < 1e-6
    assert y > 0


def test_east_offset_increases_x_only():
    x, y = latlon_to_local_xy(37.5665, 126.9790, 37.5665, 126.9780)
    assert x > 0
    assert abs(y) < 1e-6


def test_heading_north_is_yaw_90deg():
    yaw = heading_deg_to_yaw_rad(0.0)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9)


def test_heading_east_is_yaw_zero():
    yaw = heading_deg_to_yaw_rad(90.0)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_yaw_wraps_into_pi_range():
    yaw = heading_deg_to_yaw_rad(-90.0)  # 90 - (-90) = 180deg -> pi, but
    assert -math.pi <= yaw <= math.pi     # should normalize, not blow past pi


def test_quaternion_is_unit_and_zero_yaw_is_identity():
    qx, qy, qz, qw = yaw_to_quaternion_xyzw(0.0)
    assert (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 1.0)

    qx, qy, qz, qw = yaw_to_quaternion_xyzw(math.pi / 2)
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    assert math.isclose(norm, 1.0, abs_tol=1e-9)
