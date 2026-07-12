"""Builds the master /integrate/chameleon_in payload from Autoware's fused
perception messages (DetectedObjects, TrafficLightGroupArray), matching
r2lp1_planning_node's expected chameleon_in schema (see its README).

Depends on autoware_perception_msgs message types (like
cits_integration/converters.py) but not on rclpy, so it's testable in any
environment with those messages installed, without spinning a node.
"""

import math

from autoware_perception_msgs.msg import ObjectClassification, TrafficLightElement

_LABEL_TO_STRING = {
    ObjectClassification.UNKNOWN: 'unknown',
    ObjectClassification.CAR: 'car',
    ObjectClassification.TRUCK: 'truck',
    ObjectClassification.BUS: 'bus',
    ObjectClassification.MOTORCYCLE: 'motorcycle',
    ObjectClassification.BICYCLE: 'bicycle',
    ObjectClassification.PEDESTRIAN: 'pedestrian',
}
if hasattr(ObjectClassification, 'HAZARD'):
    _LABEL_TO_STRING[ObjectClassification.HAZARD] = 'hazard'

_COLOR_TO_STATE = {
    TrafficLightElement.RED: 'RED',
    TrafficLightElement.AMBER: 'YELLOW',
    TrafficLightElement.GREEN: 'GREEN',
}
# Lower number = more restrictive = wins when multiple groups disagree.
_STATE_PRIORITY = {'RED': 0, 'YELLOW': 1, 'GREEN': 2, 'UNKNOWN': 3}


def extract_worst_traffic_light_state(signals_msg):
    """Most restrictive state across every group/element, so one stale or
    unknown signal can't mask a real red reported elsewhere."""
    worst = 'UNKNOWN'
    for group in signals_msg.traffic_light_groups:
        for element in group.elements:
            state = _COLOR_TO_STATE.get(element.color, 'UNKNOWN')
            if _STATE_PRIORITY[state] < _STATE_PRIORITY[worst]:
                worst = state
    return worst


def extract_closest_hazard(objects_msg, ego_x=0.0, ego_y=0.0):
    """Nearest object to (ego_x, ego_y), or None if there are no objects.
    Ego position defaults to the map origin - a Phase 1 placeholder until
    real localization is wired in (see README)."""
    closest = None
    closest_distance = None
    for obj in objects_msg.objects:
        pos = obj.kinematics.pose_with_covariance.pose.position
        distance = math.hypot(pos.x - ego_x, pos.y - ego_y)
        if closest_distance is None or distance < closest_distance:
            closest_distance = distance
            closest = obj

    if closest is None:
        return None

    pos = closest.kinematics.pose_with_covariance.pose.position
    angle_deg = math.degrees(math.atan2(pos.y - ego_y, pos.x - ego_x))
    label = closest.classification[0].label if closest.classification else ObjectClassification.UNKNOWN

    return {
        'distance_m': closest_distance,
        'angle_deg': angle_deg,
        'object_type': _LABEL_TO_STRING.get(label, 'unknown'),
        'confidence': closest.existence_probability,
    }
