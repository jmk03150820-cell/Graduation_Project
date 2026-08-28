"""
CARLA -> common (OSI-based) coordinate & unit conversions.

Values verified against carla-simulator/ros-bridge `carla_common/transforms.py`
(the authoritative source), NOT from memory — this is the exact spot the project
got wrong three times.

CARLA native frame: left-handed (Unreal), Z-up, metres, angles in DEGREES.
Common/OSI frame:    right-handed, metres, angles in RADIANS.

Rules (from transforms.py):
  position / linear velocity / linear acceleration : negate Y only.
  angular velocity : negate Y and Z, and deg -> rad.
  orientation (roll/pitch/yaw) : roll unchanged, negate pitch and yaw, deg -> rad.

We keep orientation as RPY radians (OSI Orientation3d), skipping the euler->quat
step ros-bridge does — that is ROS-specific and OSI wants RPY, so no transforms3d.

Pure functions, no `carla` import -> testable with no simulator. The CARLA
backend just feeds `v.x, v.y, v.z` / `rot.roll, rot.pitch, rot.yaw` in.
"""
from __future__ import annotations

import math

Vec3 = tuple[float, float, float]


def location_to_common(x: float, y: float, z: float) -> Vec3:
    """carla.Location (m) -> common position (m). Negate Y."""
    return (x, -y, z)


def velocity_to_common(x: float, y: float, z: float) -> Vec3:
    """carla linear velocity (m/s) -> common (m/s). Negate Y."""
    return (x, -y, z)


def acceleration_to_common(x: float, y: float, z: float) -> Vec3:
    """carla linear acceleration (m/s^2) -> common (m/s^2). Negate Y."""
    return (x, -y, z)


def angular_velocity_to_common(x: float, y: float, z: float) -> Vec3:
    """carla angular velocity (deg/s) -> common (rad/s). Negate Y and Z, deg->rad."""
    return (math.radians(x), -math.radians(y), -math.radians(z))


def rotation_to_common(roll: float, pitch: float, yaw: float) -> Vec3:
    """
    carla.Rotation (degrees, Euler — NOT a quaternion) -> common RPY (rad).
    roll unchanged, pitch and yaw negated, deg->rad.
    """
    return (math.radians(roll), -math.radians(pitch), -math.radians(yaw))


def _demo() -> None:
    def close(a: Vec3, b: Vec3) -> bool:
        return all(math.isclose(p, q, abs_tol=1e-9) for p, q in zip(a, b))

    # linear quantities: Y flips, X/Z untouched, no unit change
    assert location_to_common(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)
    assert velocity_to_common(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)
    assert acceleration_to_common(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)

    # angular velocity: Y and Z flip, AND deg->rad (the bit the old doc got wrong)
    r = math.radians(90.0)
    assert close(angular_velocity_to_common(90.0, 90.0, 90.0), (r, -r, -r))

    # orientation: roll kept, pitch & yaw negated, deg->rad
    assert close(rotation_to_common(90.0, 90.0, 90.0), (r, -r, -r))
    # roll really is untouched in sign:
    assert close(rotation_to_common(45.0, 0.0, 0.0), (math.radians(45.0), 0.0, 0.0))

    # zero maps to zero everywhere (sanity)
    for fn in (location_to_common, velocity_to_common, acceleration_to_common,
               angular_velocity_to_common, rotation_to_common):
        assert close(fn(0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    print("transforms self-test OK")


if __name__ == "__main__":
    _demo()
