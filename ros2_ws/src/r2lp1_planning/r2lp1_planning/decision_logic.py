"""Phase 1 (if-else) planning decision logic for r2lp1_planning_node.

Kept free of rclpy so it can be unit tested directly and swapped for an
RL policy in Phase 2 without touching the node's ROS plumbing.
"""

import json
from dataclasses import dataclass
from typing import Optional


@dataclass
class Decision:
    linear_x: float
    angular_z: float
    event: str
    error: Optional[str] = None


def parse_chameleon_json(raw: str) -> dict:
    """Parses one /integrate/chameleon_in payload. Raises ValueError/JSONDecodeError."""
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError('chameleon_in payload must be a JSON object')
    return data


def decide(data: Optional[dict], params: dict) -> Decision:
    """Priority order: no/stale data -> traffic light -> hazard distance -> cruise.

    `data` is the parsed chameleon_in payload, or None when the last
    message is missing/stale - always returns a safe stop in that case.
    Expected shape (placeholder pending module_integrate_node, see README):
        {"traffic_light": {"state": "RED"|"YELLOW"|"GREEN"|"UNKNOWN"},
         "hazard": {"distance_m": float, "angle_deg": float, "object_type": str}}
    """
    if data is None:
        return Decision(0.0, 0.0, 'no_data', error='no_fresh_chameleon_in_data')

    traffic_light = (data.get('traffic_light') or {}).get('state', 'UNKNOWN')
    if traffic_light in ('RED', 'YELLOW'):
        return Decision(0.0, 0.0, f'stop_traffic_light_{traffic_light}')

    hazard = data.get('hazard') or {}
    distance = hazard.get('distance_m')
    angle = hazard.get('angle_deg', 0.0)

    if distance is not None:
        if distance <= params['emergency_stop_distance_m']:
            return Decision(0.0, 0.0, 'stop_hazard_too_close')

        if distance <= params['slow_down_distance_m']:
            span = params['slow_down_distance_m'] - params['emergency_stop_distance_m']
            ratio = (distance - params['emergency_stop_distance_m']) / span if span > 0 else 1.0
            ratio = max(0.0, min(1.0, ratio))
            speed = params['min_speed_mps'] + ratio * (
                params['target_speed_mps'] - params['min_speed_mps'])
            steer = -angle * params['steering_gain']
            steer = max(-params['max_steer_angular_z'], min(params['max_steer_angular_z'], steer))
            return Decision(speed, steer, 'slow_down_hazard')

    return Decision(params['target_speed_mps'], 0.0, 'cruise')
