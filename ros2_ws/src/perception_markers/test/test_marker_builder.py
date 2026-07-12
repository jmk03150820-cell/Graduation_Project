"""Tests for marker_builder.py. Needs autoware_perception_msgs and
visualization_msgs installed (no rclpy/node needed).

Run standalone with:  python3 -m pytest test/test_marker_builder.py
"""

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjects,
    ObjectClassification,
    Shape,
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from visualization_msgs.msg import Marker

from perception_markers.marker_builder import build_object_markers, build_traffic_light_markers


def _make_object(x, y, label=ObjectClassification.PEDESTRIAN, shape_type=Shape.BOUNDING_BOX):
    obj = DetectedObject()
    obj.existence_probability = 0.8
    classification = ObjectClassification()
    classification.label = label
    obj.classification = [classification]
    obj.kinematics.pose_with_covariance.pose.position.x = float(x)
    obj.kinematics.pose_with_covariance.pose.position.y = float(y)
    obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0
    obj.shape.type = shape_type
    obj.shape.dimensions.x = 1.0
    obj.shape.dimensions.y = 1.0
    obj.shape.dimensions.z = 1.0
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


def test_build_object_markers_emits_one_shape_and_one_label_per_object():
    msg = DetectedObjects()
    msg.header.frame_id = 'map'
    msg.objects = [_make_object(1.0, 2.0), _make_object(3.0, 4.0)]
    markers = build_object_markers(msg)
    assert len(markers.markers) == 4


def test_build_object_markers_uses_object_pose():
    msg = DetectedObjects()
    msg.header.frame_id = 'map'
    msg.objects = [_make_object(5.0, 6.0)]
    markers = build_object_markers(msg)
    shape_marker = next(m for m in markers.markers if m.type != Marker.TEXT_VIEW_FACING)
    assert shape_marker.pose.position.x == 5.0
    assert shape_marker.pose.position.y == 6.0


def test_build_object_markers_empty_objects_gives_empty_array():
    msg = DetectedObjects()
    msg.objects = []
    markers = build_object_markers(msg)
    assert markers.markers == []


def test_pedestrian_and_car_get_different_colors():
    msg = DetectedObjects()
    msg.objects = [
        _make_object(0, 0, label=ObjectClassification.PEDESTRIAN),
        _make_object(0, 0, label=ObjectClassification.CAR),
    ]
    markers = build_object_markers(msg)
    shape_markers = [m for m in markers.markers if m.type != Marker.TEXT_VIEW_FACING]
    color_a = (shape_markers[0].color.r, shape_markers[0].color.g, shape_markers[0].color.b)
    color_b = (shape_markers[1].color.r, shape_markers[1].color.g, shape_markers[1].color.b)
    assert color_a != color_b


def test_build_traffic_light_markers_one_per_group():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    assert len(markers.markers) == 2


def test_build_traffic_light_markers_are_spread_out_along_x():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    xs = sorted(m.pose.position.x for m in markers.markers)
    assert xs[0] != xs[1]


def test_build_traffic_light_markers_red_and_green_differ_in_color():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    colors = {(m.color.r, m.color.g, m.color.b) for m in markers.markers}
    assert len(colors) == 2


def test_build_traffic_light_markers_no_groups_gives_empty_array():
    msg = _make_signals()
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    assert markers.markers == []
