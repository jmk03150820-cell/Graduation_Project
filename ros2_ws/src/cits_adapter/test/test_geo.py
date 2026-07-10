"""Pure-Python tests for geo.py; no ROS/Autoware install required.

Run standalone with:  python3 -m pytest test/test_geo.py
"""

import math

from cits_adapter.geo import (
    bev_heading_to_map_yaw_rad,
    bev_to_map_xy,
    yaw_to_quaternion_xyzw,
)


def test_rsu_at_origin_no_rotation_is_identity():
    x, y = bev_to_map_xy(3.0, 4.0, 0.0, 0.0, 0.0)
    assert math.isclose(x, 3.0)
    assert math.isclose(y, 4.0)


def test_translation_only():
    x, y = bev_to_map_xy(1.0, 2.0, 10.0, 20.0, 0.0)
    assert math.isclose(x, 11.0)
    assert math.isclose(y, 22.0)


def test_90deg_rsu_yaw_rotates_forward_axis_to_map_y():
    # RSU facing +90deg (map yaw): its own +x (forward) should map to +y.
    x, y = bev_to_map_xy(1.0, 0.0, 0.0, 0.0, 90.0)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 1.0, abs_tol=1e-9)


def test_heading_adds_rsu_yaw_offset():
    yaw = bev_heading_to_map_yaw_rad(0.0, 90.0)
    assert math.isclose(yaw, math.radians(90.0), abs_tol=1e-9)


def test_heading_wraps_into_pi_range():
    yaw = bev_heading_to_map_yaw_rad(170.0, 170.0)  # sum = 340deg -> wraps
    assert -math.pi <= yaw <= math.pi
    assert math.isclose(yaw, math.radians(340.0 - 360.0), abs_tol=1e-9)


def test_quaternion_zero_yaw_is_identity():
    assert yaw_to_quaternion_xyzw(0.0) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_is_unit_norm():
    qx, qy, qz, qw = yaw_to_quaternion_xyzw(math.radians(37.0))
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    assert math.isclose(norm, 1.0, abs_tol=1e-9)
