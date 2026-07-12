"""Pure-Python generator for fake RSU-detected objects, matching the
predicted cits_rsu_msgs/DetectedObject schema (the C-ITS teammate's field
table: object_id/object_type/position_x/position_y/velocity/timestamp/
confidence/heading). No ROS imports, so testable without ROS installed.

Simulates two objects in a BEV frame centered on the RSU, positioned near
the same intersection perception_markers draws (traffic lights at x=10,
lanes at y=-2/y=+2 - see perception_markers/config), so they visually
read as part of the scene instead of floating off to the side:
- a pedestrian crossing the road at the crosswalk near the stop line
- a car driving along one of the lanes
"""

import math

# Mirrors cits_rsu_msgs/msg/DetectedObject object_type constants.
PERSON = 1
CAR = 2


def _oscillate(now, period_s, amplitude):
    """Ping-pong position in [-amplitude, amplitude] and its signed velocity
    direction (+1 moving positive, -1 moving negative)."""
    t = now % period_s
    half = period_s / 2.0
    if t < half:
        frac = t / half
        pos = -amplitude + 2 * amplitude * frac
        direction = 1.0
    else:
        frac = (t - half) / half
        pos = amplitude - 2 * amplitude * frac
        direction = -1.0
    return pos, direction


def generate_objects(now, pedestrian_id=1, car_id=2):
    """Returns a list of dicts, each matching the DetectedObject field set."""
    now_ms = int(now * 1000)

    # Crosswalk near the intersection stop line (x=10 - see
    # perception_markers' traffic_light_x), sweeping across the road width.
    ped_offset, ped_dir = _oscillate(now, period_s=10.0, amplitude=3.0)
    pedestrian = {
        'object_id': pedestrian_id,
        'object_type': PERSON,
        'position_x': 10.0 + ped_offset,
        'position_y': 0.0,
        'velocity': 1.4,
        'timestamp': now_ms,
        'confidence': 0.9,
        'heading': 0.0 if ped_dir > 0 else 180.0,
    }

    # Driving along the y=-2 lane (same lane as traffic-light group 1).
    car_x, car_dir = _oscillate(now, period_s=20.0, amplitude=30.0)
    car = {
        'object_id': car_id,
        'object_type': CAR,
        'position_x': car_x,
        'position_y': -2.0,
        'velocity': 8.0,
        'timestamp': now_ms,
        'confidence': 0.95,
        'heading': 90.0 if car_dir > 0 else 270.0,
    }

    return [pedestrian, car]
