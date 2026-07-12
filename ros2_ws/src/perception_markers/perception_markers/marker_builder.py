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
_UNKNOWN_TRAFFIC_LIGHT_COLOR = (0.5, 0.5, 0.5, 0.6)


def _set_color(marker, rgba):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


def _set_lifetime(marker, lifetime_s):
    marker.lifetime.sec = int(lifetime_s)
    marker.lifetime.nanosec = int((lifetime_s - int(lifetime_s)) * 1e9)


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
                                 lifetime_s=0.5):
    """One SPHERE per signal group, spread out along x so groups don't
    overlap. Position is a fixed placeholder, not each group's real
    location - TrafficLightGroupArray only carries a group id (it expects
    the real location to come from a lanelet2 map's regulatory elements,
    which this project doesn't have yet). See README."""
    markers = []
    x, y, z = position_xyz
    for i, group in enumerate(signals_msg.traffic_light_groups):
        color = _UNKNOWN_TRAFFIC_LIGHT_COLOR
        for element in group.elements:
            if element.color in _TRAFFIC_LIGHT_COLOR:
                color = _TRAFFIC_LIGHT_COLOR[element.color]
                break

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = signals_msg.stamp
        marker.ns = ns
        marker.id = group.traffic_light_group_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x + i * 1.5
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.8
        _set_color(marker, color)
        _set_lifetime(marker, lifetime_s)
        markers.append(marker)

    return MarkerArray(markers=markers)
