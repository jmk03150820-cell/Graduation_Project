"""Tests for marker_builder.py. Needs autoware_perception_msgs and
visualization_msgs installed (no rclpy/node needed).

Run standalone with:  python3 -m pytest test/test_marker_builder.py
"""

import math

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

from perception_markers.marker_builder import (
    build_lane_vehicle_markers,
    build_object_markers,
    build_road_markers,
    build_traffic_light_markers,
)


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


def test_build_traffic_light_markers_one_sphere_and_one_label_per_group():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    assert len(markers.markers) == 4
    spheres = [m for m in markers.markers if m.type == Marker.SPHERE]
    labels = [m for m in markers.markers if m.type == Marker.TEXT_VIEW_FACING]
    assert len(spheres) == 2
    assert len(labels) == 2


def test_build_traffic_light_markers_are_spread_across_lanes_not_x():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0), lane_spacing_m=4.0)
    spheres = [m for m in markers.markers if m.type == Marker.SPHERE]
    xs = {m.pose.position.x for m in spheres}
    ys = sorted(m.pose.position.y for m in spheres)
    assert xs == {10.0}  # same stop-line distance for every lane
    assert math.isclose(ys[1] - ys[0], 4.0)


def test_build_traffic_light_markers_lanes_are_centered_on_y():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 100.0, 3.0), lane_spacing_m=4.0)
    spheres = [m for m in markers.markers if m.type == Marker.SPHERE]
    avg_y = sum(m.pose.position.y for m in spheres) / len(spheres)
    assert math.isclose(avg_y, 100.0)


def test_build_traffic_light_markers_lane_assignment_is_stable_regardless_of_message_order():
    msg = _make_signals(TrafficLightElement.GREEN, TrafficLightElement.RED)  # group 0 green, group 1 red
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0), lane_spacing_m=4.0)
    spheres = {m.id: m for m in markers.markers if m.type == Marker.SPHERE}
    assert spheres[0].pose.position.y < spheres[1].pose.position.y


def test_build_traffic_light_markers_red_and_green_differ_in_color():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    spheres = [m for m in markers.markers if m.type == Marker.SPHERE]
    colors = {(m.color.r, m.color.g, m.color.b) for m in spheres}
    assert len(colors) == 2


def test_build_traffic_light_markers_labels_include_group_id_and_state():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    labels = {m.text for m in markers.markers if m.type == Marker.TEXT_VIEW_FACING}
    assert labels == {'group 0: RED', 'group 1: GREEN'}


def test_build_traffic_light_markers_no_groups_gives_empty_array():
    msg = _make_signals()
    markers = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0))
    assert markers.markers == []


def test_lane_vehicle_markers_one_per_group():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_lane_vehicle_markers(msg, 'map', (10.0, 0.0, 3.0))
    assert len(markers.markers) == 2


def test_lane_vehicle_markers_share_lane_y_with_their_own_light():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    lights = build_traffic_light_markers(msg, 'map', (10.0, 0.0, 3.0), lane_spacing_m=4.0)
    cars = build_lane_vehicle_markers(msg, 'map', (10.0, 0.0, 3.0), lane_spacing_m=4.0)
    light_y_by_id = {m.id: m.pose.position.y for m in lights.markers if m.type == Marker.SPHERE}
    for car in cars.markers:
        assert math.isclose(car.pose.position.y, light_y_by_id[car.id])


def test_lane_vehicle_markers_are_behind_the_stop_line():
    msg = _make_signals(TrafficLightElement.RED)
    markers = build_lane_vehicle_markers(msg, 'map', (10.0, 0.0, 3.0), approach_distance_m=6.0)
    assert markers.markers[0].pose.position.x == 4.0


def test_lane_vehicle_markers_red_is_stopped_green_is_moving():
    msg = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    markers = build_lane_vehicle_markers(msg, 'map', (10.0, 0.0, 3.0))
    cars = {m.id: m for m in markers.markers}
    assert cars[0].color.r > cars[0].color.g  # red group -> reddish car
    assert cars[1].color.g > cars[1].color.r  # green group -> greenish car


def test_lane_vehicle_markers_no_groups_gives_empty_array():
    msg = _make_signals()
    markers = build_lane_vehicle_markers(msg, 'map', (10.0, 0.0, 3.0))
    assert markers.markers == []


def test_road_markers_two_lanes_gives_three_lines_plus_stop_line():
    stamp = TrafficLightGroupArray().stamp
    markers = build_road_markers('map', stamp, (10.0, 0.0, 3.0), num_lanes=2, lane_spacing_m=4.0)
    assert len(markers.markers) == 4  # 2 edges + 1 center divider + 1 stop line


def test_road_markers_center_line_is_yellow_edges_are_white():
    stamp = TrafficLightGroupArray().stamp
    markers = build_road_markers('map', stamp, (10.0, 0.0, 3.0), num_lanes=2, lane_spacing_m=4.0)
    lane_lines = [m for m in markers.markers if m.ns == 'road']
    center = next(m for m in lane_lines if math.isclose(m.pose.position.y, 0.0))
    edges = [m for m in lane_lines if not math.isclose(m.pose.position.y, 0.0)]
    assert center.color.g > center.color.b  # yellow-ish
    for edge in edges:
        assert math.isclose(edge.color.r, edge.color.g)  # white


def test_road_markers_lane_lines_match_traffic_light_lane_positions():
    stamp = TrafficLightGroupArray().stamp
    signals = _make_signals(TrafficLightElement.RED, TrafficLightElement.GREEN)
    lights = build_traffic_light_markers(signals, 'map', (10.0, 0.0, 3.0), lane_spacing_m=4.0)
    road = build_road_markers('map', stamp, (10.0, 0.0, 3.0), num_lanes=2, lane_spacing_m=4.0)
    light_ys = sorted(m.pose.position.y for m in lights.markers if m.type == Marker.SPHERE)
    lane_line_ys = sorted(m.pose.position.y for m in road.markers if m.ns == 'road')
    # each light sits between two of the drawn lane lines (its own lane)
    assert lane_line_ys[0] < light_ys[0] < lane_line_ys[1]
    assert lane_line_ys[1] < light_ys[1] < lane_line_ys[2]


def test_road_markers_stop_line_is_at_the_intersection_x():
    stamp = TrafficLightGroupArray().stamp
    markers = build_road_markers('map', stamp, (10.0, 0.0, 3.0), num_lanes=2)
    stop_line = next(m for m in markers.markers if m.ns == 'road_stop_line')
    assert stop_line.pose.position.x == 10.0
