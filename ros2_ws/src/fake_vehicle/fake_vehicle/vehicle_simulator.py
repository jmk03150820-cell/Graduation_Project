"""Pure-Python fake vehicle physics, no ROS imports so it is testable
without ROS installed.

Simulates rate-limited tracking of a target speed/steer command, standing
in for real hardware inertia until a real vehicle is connected.
"""


class VehicleSimulator:
    """Stateful speed/steer tracker with acceleration and steer-rate limits."""

    def __init__(self, max_accel_mps2=2.0, max_decel_mps2=3.0, max_steer_rate_deg_s=45.0):
        self.speed_kmh = 0.0
        self.steer_deg = 0.0
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

        return self.speed_kmh, self.steer_deg
