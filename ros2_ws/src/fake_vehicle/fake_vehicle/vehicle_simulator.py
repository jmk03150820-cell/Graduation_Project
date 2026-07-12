"""Pure-Python fake vehicle physics, no ROS imports so it is testable
without ROS installed.

Simulates rate-limited tracking of a target speed/steer command, standing
in for real hardware inertia until a real vehicle is connected. Also dead-
reckons a map-frame pose so the simulated motion can be drawn/moved in a
3D view - see README for the simplified (non-bicycle-model) steering.
"""

import math


class VehicleSimulator:
    """Stateful speed/steer tracker with acceleration and steer-rate limits,
    plus a dead-reckoned 2D pose (x, y, yaw_deg) for visualization."""

    def __init__(self, max_accel_mps2=2.0, max_decel_mps2=3.0, max_steer_rate_deg_s=45.0):
        self.speed_kmh = 0.0
        self.steer_deg = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw_deg = 0.0
        self._max_accel_kmh_s = max_accel_mps2 * 3.6
        self._max_decel_kmh_s = max_decel_mps2 * 3.6
        self._max_steer_rate_deg_s = max_steer_rate_deg_s

    def step(self, dt_s, target_speed_kmh, target_steer_deg):
        """Advances the simulated vehicle by dt_s toward the given targets.

        No reverse in Phase 1 - speed is clamped to >= 0. Returns the new
        (speed_kmh, steer_deg).
        """
        speed_diff = target_speed_kmh - self.speed_kmh
        max_delta = (self._max_accel_kmh_s if speed_diff > 0 else self._max_decel_kmh_s) * dt_s
        if abs(speed_diff) <= max_delta:
            self.speed_kmh = target_speed_kmh
        else:
            self.speed_kmh += max_delta if speed_diff > 0 else -max_delta
        self.speed_kmh = max(0.0, self.speed_kmh)

        steer_diff = target_steer_deg - self.steer_deg
        max_steer_delta = self._max_steer_rate_deg_s * dt_s
        if abs(steer_diff) <= max_steer_delta:
            self.steer_deg = target_steer_deg
        else:
            self.steer_deg += max_steer_delta if steer_diff > 0 else -max_steer_delta

        # Dead-reckon a map-frame pose for visualization. Phase 1
        # simplification: steer_deg is treated directly as a yaw rate
        # (deg/s), not a real steering-wheel angle through a bicycle
        # model - good enough to show "it moves, then stops", not for
        # trajectory accuracy.
        self.yaw_deg += self.steer_deg * dt_s
        yaw_rad = math.radians(self.yaw_deg)
        speed_mps = self.speed_kmh / 3.6
        self.x += speed_mps * math.cos(yaw_rad) * dt_s
        self.y += speed_mps * math.sin(yaw_rad) * dt_s

        return self.speed_kmh, self.steer_deg
