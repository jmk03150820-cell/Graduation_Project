"""Tests for chameleon_builder.py. Needs autoware_perception_msgs
installed (no rclpy/node needed though).

Run standalone with:  python3 -m pytest test/test_chameleon_builder.py
"""

import math

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjects,
    ObjectClassification,
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)

from module_integrate.chameleon_builder import (
    extract_closest_hazard,
    extract_worst_traffic_light_state,
)


def _make_object(x, y, label=ObjectClassification.PEDESTRIAN, confidence=0.9):
    obj = DetectedObject()
    obj.existence_probability = confidence
    classification = ObjectClassification()
    classification.label = label
    classification.probability = 1.0
    obj.classification = [classification]
    obj.kinematics.pose_with_covariance.pose.position.x = float(x)
    obj.kinematics.pose_with_covariance.pose.position.y = float(y)
    return obj


def _make_signals(*colors):
    msg = TrafficLightGroupArray()
    groups = []
    for i, color in enumerate(colors):
        group = TrafficLightGroup()
        group.traffic_light_group_id = i
        element = TrafficLightElement()
        element.color = color
        group.elements = [element]
        groups.append(group)
    msg.traffic_light_groups = groups
    return msg


def test_extract_closest_hazard_picks_nearest_object():
    msg = DetectedObjects()
    msg.objects = [_make_object(20.0, 0.0), _make_object(5.0, 0.0)]
    hazard = extract_closest_hazard(msg)
    assert math.isclose(hazard['distance_m'], 5.0)
    assert hazard['object_type'] == 'pedestrian'


def test_extract_closest_hazard_returns_none_when_no_objects():
    msg = DetectedObjects()
    msg.objects = []
    assert extract_closest_hazard(msg) is None


def test_extract_closest_hazard_angle_is_relative_to_ego():
    msg = DetectedObjects()
    msg.objects = [_make_object(0.0, 10.0)]
    hazard = extract_closest_hazard(msg, ego_x=0.0, ego_y=0.0)
    assert math.isclose(hazard['angle_deg'], 90.0)


def test_extract_closest_hazard_uses_ego_offset():
    msg = DetectedObjects()
    msg.objects = [_make_object(10.0, 10.0)]
    hazard = extract_closest_hazard(msg, ego_x=10.0, ego_y=0.0)
    assert math.isclose(hazard['distance_m'], 10.0)


def test_worst_traffic_light_state_prefers_red_over_green():
    msg = _make_signals(TrafficLightElement.GREEN, TrafficLightElement.RED)
    assert extract_worst_traffic_light_state(msg) == 'RED'


def test_worst_traffic_light_state_prefers_yellow_over_green():
    msg = _make_signals(TrafficLightElement.GREEN, TrafficLightElement.AMBER)
    assert extract_worst_traffic_light_state(msg) == 'YELLOW'


def test_worst_traffic_light_state_all_green_is_green():
    msg = _make_signals(TrafficLightElement.GREEN, TrafficLightElement.GREEN)
    assert extract_worst_traffic_light_state(msg) == 'GREEN'


def test_worst_traffic_light_state_no_groups_is_unknown():
    msg = _make_signals()
    assert extract_worst_traffic_light_state(msg) == 'UNKNOWN'
