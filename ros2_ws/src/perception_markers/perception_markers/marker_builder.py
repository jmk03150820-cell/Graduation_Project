"""Builds visualization_msgs/MarkerArray from Autoware's fused perception
messages (DetectedObjects, TrafficLightGroupArray), so tools that don't
understand Autoware's custom message types natively (Foxglove, RViz) can
render them in a 3D panel instead of only showing raw/unrendered data.

Depends on autoware_perception_msgs/visualization_msgs types (like
chameleon_builder.py, decision_logic.py's siblings) but not on rclpy.
"""

from autoware_perception_msgs.msg import ObjectClassification, Shape, TrafficLightElement
from visualization_msgs.msg import Marker, MarkerArray

_CLASSIFICATION_COLOR = {
    ObjectClassification.PEDESTRIAN: (1.0, 0.9, 0.1, 0.85),
    ObjectClassification.BICYCLE: (0.1, 0.7, 1.0, 0.85),
    ObjectClassification.MOTORCYCLE: (0.1, 0.7, 1.0, 0.85),
    ObjectClassification.CAR: (0.2, 0.4, 1.0, 0.85),
    ObjectClassification.TRUCK: (0.2, 0.4, 1.0, 0.85),
    ObjectClassification.BUS: (0.2, 0.4, 1.0, 0.85),
}
if hasattr(ObjectClassification, 'HAZARD'):
    _CLASSIFICATION_COLOR[ObjectClassification.HAZARD] = (1.0, 0.15, 0.1, 0.85)
_DEFAULT_OBJECT_COLOR = (0.6, 0.6, 0.6, 0.7)

_LABEL_NAMES = {
    ObjectClassification.UNKNOWN: 'unknown',
    ObjectClassification.CAR: 'car',
    ObjectClassification.TRUCK: 'truck',
    ObjectClassification.BUS: 'bus',
    ObjectClassification.MOTORCYCLE: 'motorcycle',
    ObjectClassification.BICYCLE: 'bicycle',
    ObjectClassification.PEDESTRIAN: 'pedestrian',
}
if hasattr(ObjectClassification, 'HAZARD'):
    _LABEL_NAMES[ObjectClassification.HAZARD] = 'hazard'

_SHAPE_TO_MARKER_TYPE = {
    Shape.BOUNDING_BOX: Marker.CUBE,
    Shape.CYLINDER: Marker.CYLINDER,
}

_TRAFFIC_LIGHT_COLOR = {
    TrafficLightElement.RED: (1.0, 0.1, 0.1, 0.9),
    TrafficLightElement.AMBER: (1.0, 0.8, 0.0, 0.9),
    TrafficLightElement.GREEN: (0.1, 1.0, 0.2, 0.9),
}
_TRAFFIC_LIGHT_STATE_NAME = {
    TrafficLightElement.RED: 'RED',
    TrafficLightElement.AMBER: 'YELLOW',
    TrafficLightElement.GREEN: 'GREEN',
}
_UNKNOWN_TRAFFIC_LIGHT_COLOR = (0.5, 0.5, 0.5, 0.6)


def _set_color(marker, rgba):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


def _set_lifetime(marker, lifetime_s):
    marker.lifetime.sec = int(lifetime_s)
    marker.lifetime.nanosec = int((lifetime_s - int(lifetime_s)) * 1e9)


def _lane_y(index, count, center_y, spacing_m):
    """Shared lane-slot rule: index 0..count-1, centered on center_y,
    spacing_m apart. Used for both the traffic-light sphere and its demo
    car so they always land in the same lane."""
    return center_y + (index - (count - 1) / 2.0) * spacing_m


def _state_name_and_color(group):
    for element in group.elements:
        if element.color in _TRAFFIC_LIGHT_COLOR:
            return _TRAFFIC_LIGHT_STATE_NAME[element.color], _TRAFFIC_LIGHT_COLOR[element.color]
    return 'UNKNOWN', _UNKNOWN_TRAFFIC_LIGHT_COLOR


def build_object_markers(objects_msg, ns='perception_objects', lifetime_s=0.5):
    """One CUBE/CYLINDER marker per object at its actual pose/shape, plus a
    TEXT marker above it with its type and confidence, colored by
    classification. A short lifetime expires markers for objects that
    disappear next frame instead of tracking ids to send DELETE actions."""
    markers = []
    for i, obj in enumerate(objects_msg.objects):
        label = obj.classification[0].label if obj.classification else ObjectClassification.UNKNOWN
        color = _CLASSIFICATION_COLOR.get(label, _DEFAULT_OBJECT_COLOR)
        pose = obj.kinematics.pose_with_covariance.pose

        marker = Marker()
        marker.header = objects_msg.header
        marker.ns = ns
        marker.id = i
        marker.type = _SHAPE_TO_MARKER_TYPE.get(obj.shape.type, Marker.CUBE)
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = max(obj.shape.dimensions.x, 0.1)
        marker.scale.y = max(obj.shape.dimensions.y, 0.1)
        marker.scale.z = max(obj.shape.dimensions.z, 0.1)
        _set_color(marker, color)
        _set_lifetime(marker, lifetime_s)
        markers.append(marker)

        text = Marker()
        text.header = objects_msg.header
        text.ns = ns + '_label'
        text.id = i
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = pose
        text.pose.position.z = pose.position.z + obj.shape.dimensions.z / 2.0 + 0.5
        text.scale.z = 0.6
        _set_color(text, (1.0, 1.0, 1.0, 0.9))
        text.text = f'{_LABEL_NAMES.get(label, "unknown")} {obj.existence_probability:.2f}'
        _set_lifetime(text, lifetime_s)
        markers.append(text)

    return MarkerArray(markers=markers)


def build_traffic_light_markers(signals_msg, frame_id, position_xyz, ns='traffic_lights',
                                 lifetime_s=0.5, lane_spacing_m=4.0):
    """One SPHERE per signal group plus a TEXT label ("group N: STATE")
    above it. All groups sit at the same x/z (the intersection's stop
    line, straight ahead of ego) and are laid out one per lane: sorted by
    group id, spread along y and centered on position_xyz's y, spacing_m
    apart - e.g. 2 groups become left-lane/right-lane instead of two
    spheres stacked with no visible rule. An intersection normally has
    several independent signal groups (e.g. one per direction), so more
    than one sphere at once is expected, not a bug.

    Position is still a fixed placeholder, not each group's real
    location - TrafficLightGroupArray only carries a group id (it expects
    the real location to come from a lanelet2 map's regulatory elements,
    which this project doesn't have yet). See README."""
    markers = []
    x, y, z = position_xyz
    groups = sorted(signals_msg.traffic_light_groups, key=lambda g: g.traffic_light_group_id)
    n = len(groups)

    for i, group in enumerate(groups):
        state_name, color = _state_name_and_color(group)
        marker_y = _lane_y(i, n, y, lane_spacing_m)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = signals_msg.stamp
        marker.ns = ns
        marker.id = group.traffic_light_group_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = marker_y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.8
        _set_color(marker, color)
        _set_lifetime(marker, lifetime_s)
        markers.append(marker)

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = signals_msg.stamp
        text.ns = ns + '_label'
        text.id = group.traffic_light_group_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = marker_y
        text.pose.position.z = z + 0.7
        text.pose.orientation.w = 1.0
        text.scale.z = 0.5
        _set_color(text, (1.0, 1.0, 1.0, 0.9))
        text.text = f'group {group.traffic_light_group_id}: {state_name}'
        _set_lifetime(text, lifetime_s)
        markers.append(text)

    return MarkerArray(markers=markers)


# Car-sized box, matching fake_vehicle_node's ego marker dimensions.
_LANE_CAR_SCALE = (4.5, 1.9, 1.8)
_LANE_CAR_STOPPED_COLOR = (1.0, 0.15, 0.15, 0.9)
_LANE_CAR_MOVING_COLOR = (0.15, 0.85, 0.25, 0.9)
_LANE_CAR_UNKNOWN_COLOR = (0.6, 0.6, 0.6, 0.7)


def build_lane_vehicle_markers(signals_msg, frame_id, position_xyz, lane_spacing_m=4.0,
                                approach_distance_m=6.0, ns='lane_vehicles', lifetime_s=0.5):
    """One demo car per signal group, parked in that group's own lane
    (same _lane_y rule as build_traffic_light_markers) just behind the
    stop line - colored red for RED/YELLOW (stopped) or green for GREEN
    (would be going), directly from that lane's own state only. This is
    an illustrative, independent-per-lane demo: it does NOT go through
    module_integrate/r2lp1_planning (which correctly reacts to only the
    single worst state across all lanes for one real ego vehicle) - it
    exists purely so each light's effect is visually unambiguous."""
    markers = []
    x, y, z = position_xyz
    groups = sorted(signals_msg.traffic_light_groups, key=lambda g: g.traffic_light_group_id)
    n = len(groups)
    car_z = _LANE_CAR_SCALE[2] / 2.0

    for i, group in enumerate(groups):
        state_name, _ = _state_name_and_color(group)
        if state_name == 'GREEN':
            color = _LANE_CAR_MOVING_COLOR
        elif state_name in ('RED', 'YELLOW'):
            color = _LANE_CAR_STOPPED_COLOR
        else:
            color = _LANE_CAR_UNKNOWN_COLOR

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = signals_msg.stamp
        marker.ns = ns
        marker.id = group.traffic_light_group_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x - approach_distance_m
        marker.pose.position.y = _lane_y(i, n, y, lane_spacing_m)
        marker.pose.position.z = car_z
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = _LANE_CAR_SCALE
        _set_color(marker, color)
        _set_lifetime(marker, lifetime_s)
        markers.append(marker)

    return MarkerArray(markers=markers)


_LANE_LINE_COLOR = (1.0, 1.0, 1.0, 0.9)
_CENTER_LINE_COLOR = (1.0, 0.85, 0.1, 0.9)
_STOP_LINE_COLOR = (1.0, 1.0, 1.0, 0.95)
_LINE_WIDTH_M = 0.15
_LINE_THICKNESS_M = 0.02


def build_road_markers(frame_id, stamp, position_xyz, num_lanes, lane_spacing_m=4.0,
                        road_start_x=-10.0, road_end_x=30.0, ns='road', lifetime_s=0.5):
    """Flat CUBE markers for the road ego and the lane demo cars sit on:
    one line per lane boundary (n+1 lines for n lanes - the middle one(s)
    yellow like a real center line, the two outer edges white), plus one
    white stop line across the road at the intersection's x. Static scene
    decoration so the lanes/stop-line implied by build_traffic_light_markers
    and build_lane_vehicle_markers are actually visible, not just implied
    by marker positions floating in empty space."""
    x_stop, y_center, _ = position_xyz
    road_length = road_end_x - road_start_x
    road_center_x = (road_start_x + road_end_x) / 2.0
    markers = []

    for i in range(num_lanes + 1):
        line_y = y_center + (i - num_lanes / 2.0) * lane_spacing_m
        is_center = num_lanes % 2 == 0 and i == num_lanes // 2

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = road_center_x
        marker.pose.position.y = line_y
        marker.pose.position.z = _LINE_THICKNESS_M / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = (
            road_length, _LINE_WIDTH_M, _LINE_THICKNESS_M)
        _set_color(marker, _CENTER_LINE_COLOR if is_center else _LANE_LINE_COLOR)
        _set_lifetime(marker, lifetime_s)
        markers.append(marker)

    stop_line = Marker()
    stop_line.header.frame_id = frame_id
    stop_line.header.stamp = stamp
    stop_line.ns = ns + '_stop_line'
    stop_line.id = 0
    stop_line.type = Marker.CUBE
    stop_line.action = Marker.ADD
    stop_line.pose.position.x = x_stop
    stop_line.pose.position.y = y_center
    stop_line.pose.position.z = _LINE_THICKNESS_M / 2.0
    stop_line.pose.orientation.w = 1.0
    stop_line.scale.x, stop_line.scale.y, stop_line.scale.z = (
        0.3, num_lanes * lane_spacing_m + _LINE_WIDTH_M, _LINE_THICKNESS_M)
    _set_color(stop_line, _STOP_LINE_COLOR)
    _set_lifetime(stop_line, lifetime_s)
    markers.append(stop_line)

    return MarkerArray(markers=markers)
