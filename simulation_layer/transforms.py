"""CARLA -> common (right-handed ENU, SI, radian) coordinate & unit conversions.

Conversion rules verified against carla-simulator/ros-bridge
`carla_common/transforms.py` (sim_backend_adapter/transforms.py에서 이전, 검증 이력 유지):

CARLA native frame: left-handed (Unreal), Z-up, metres, angles in DEGREES.
  position / linear velocity / linear acceleration : negate Y only.
  angular velocity : negate Y and Z, deg -> rad.
  orientation (roll/pitch/yaw) : roll unchanged, negate pitch and yaw, deg -> rad.

Pure functions, no `carla` import — testable with no simulator.
"""
from __future__ import annotations

import math

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # x, y, z, w


def location_to_common(x: float, y: float, z: float) -> Vec3:
    return (x, -y, z)


def velocity_to_common(x: float, y: float, z: float) -> Vec3:
    return (x, -y, z)


def acceleration_to_common(x: float, y: float, z: float) -> Vec3:
    return (x, -y, z)


def angular_velocity_to_common(x: float, y: float, z: float) -> Vec3:
    """carla angular velocity (deg/s) -> common (rad/s). Negate Y and Z, deg->rad."""
    return (math.radians(x), -math.radians(y), -math.radians(z))


def rotation_to_common_rpy(roll: float, pitch: float, yaw: float) -> Vec3:
    """carla.Rotation (degrees) -> common RPY (rad). roll kept, pitch/yaw negated."""
    return (math.radians(roll), -math.radians(pitch), -math.radians(yaw))


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quat:
    """common RPY (rad) -> quaternion x,y,z,w (extrinsic XYZ = intrinsic ZYX)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rotation_to_common_quaternion(roll: float, pitch: float, yaw: float) -> Quat:
    """carla.Rotation (degrees) -> common quaternion x,y,z,w."""
    return rpy_to_quaternion(*rotation_to_common_rpy(roll, pitch, yaw))


def steering_rad_to_native(angle_rad: float, max_steer_angle_rad: float) -> float:
    """공통 조향각(rad, 좌회전 양수) -> CARLA normalized steer [-1, 1].

    CARLA steer는 우회전 양수(left-handed)라 부호를 뒤집는다. max_steer_angle은
    spawn 시 wheel physics에서 1회 캐싱한 값(§2-5 ⑤).
    """
    if max_steer_angle_rad <= 0.0:
        raise ValueError("max_steer_angle_rad must be positive")
    return max(-1.0, min(1.0, -angle_rad / max_steer_angle_rad))


def _demo() -> None:
    def close(a, b) -> bool:
        return all(math.isclose(p, q, abs_tol=1e-9) for p, q in zip(a, b))

    assert location_to_common(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)
    r = math.radians(90.0)
    assert close(angular_velocity_to_common(90.0, 90.0, 90.0), (r, -r, -r))
    assert close(rotation_to_common_rpy(90.0, 90.0, 90.0), (r, -r, -r))

    # quaternion: zero rotation -> identity
    assert close(rpy_to_quaternion(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    # yaw 90° -> (0,0,sin45,cos45)
    assert close(rpy_to_quaternion(0.0, 0.0, math.pi / 2),
                 (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    # unit norm for arbitrary rotation
    q = rpy_to_quaternion(0.3, -0.7, 2.1)
    assert math.isclose(sum(c * c for c in q), 1.0, abs_tol=1e-12)

    # steering: left-positive common -> CARLA sign flip + normalization + clamp
    assert steering_rad_to_native(0.0, 1.0) == 0.0
    assert steering_rad_to_native(0.5, 1.0) == -0.5
    assert steering_rad_to_native(-2.0, 1.0) == 1.0  # clamped
    print("transforms self-test OK")


if __name__ == "__main__":
    _demo()
