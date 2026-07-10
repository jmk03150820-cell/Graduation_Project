"""Converts parsed fake_rsu SPAT/DENM JSON dicts into Autoware Universe
(Humble) perception messages. CAM/object-detection conversion now lives in
cits_adapter (fed by the C-ITS RSU's predicted schema) instead of here.

Field names verified against the installed ros-humble-autoware-perception-msgs
via `ros2 interface show` (DetectedObject/DetectedObjects/DetectedObjectKinematics/
Shape/ObjectClassification/TrafficLightGroupArray/TrafficLightGroup/TrafficLightElement).
"""

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjectKinematics,
    DetectedObjects,
    ObjectClassification,
    Shape,
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header

from cits_integration.geo import latlon_to_local_xy

_COLOR_MAP = {
    'red': TrafficLightElement.RED,
    'yellow': TrafficLightElement.AMBER,
    'green': TrafficLightElement.GREEN,
}


def _stamp_from_unix(now_unix):
    stamp = TimeMsg()
    stamp.sec = int(now_unix)
    stamp.nanosec = int((now_unix - int(now_unix)) * 1e9)
    return stamp


def denm_to_detected_object(denm, origin_lat, origin_lon,
                             hazard_size_m=2.0, hazard_height_m=1.0):
    """An active DENM hazard event -> a stationary UNKNOWN obstacle, so
    Autoware's existing obstacle-avoidance/stop planning reacts to it
    without needing bespoke DENM-aware planning logic."""
    pos = denm['position']
    x, y = latlon_to_local_xy(pos['lat'], pos['lon'], origin_lat, origin_lon)

    obj = DetectedObject()
    obj.existence_probability = 0.99

    classification = ObjectClassification()
    classification.label = ObjectClassification.HAZARD
    classification.probability = 1.0
    obj.classification = [classification]

    k = obj.kinematics
    k.pose_with_covariance.pose.position.x = x
    k.pose_with_covariance.pose.position.y = y
    k.pose_with_covariance.pose.position.z = 0.0
    k.pose_with_covariance.pose.orientation.w = 1.0
    k.has_position_covariance = False
    k.orientation_availability = DetectedObjectKinematics.UNAVAILABLE
    k.has_twist = False
    k.has_twist_covariance = False

    obj.shape.type = Shape.CYLINDER
    obj.shape.dimensions.x = hazard_size_m
    obj.shape.dimensions.y = hazard_size_m
    obj.shape.dimensions.z = hazard_height_m

    return obj


def build_detected_objects(objects, frame_id, stamp_unix):
    msg = DetectedObjects()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = _stamp_from_unix(stamp_unix)
    msg.objects = objects
    return msg


def spat_to_traffic_light_group_array(spat, stamp_unix):
    """fake_rsu's signal_group_id is passed straight through as
    traffic_light_group_id. In a real deployment this must match the
    regulatory-element IDs in your lanelet2 map — with no real map yet,
    it's a placeholder that the Adapter/map layer will need to reconcile
    once a lanelet2 map exists."""
    msg = TrafficLightGroupArray()
    msg.stamp = _stamp_from_unix(stamp_unix)

    groups = []
    for sg in spat['signal_groups']:
        group = TrafficLightGroup()
        group.traffic_light_group_id = sg['signal_group_id']

        element = TrafficLightElement()
        element.color = _COLOR_MAP.get(sg['state'], TrafficLightElement.UNKNOWN)
        element.shape = TrafficLightElement.CIRCLE
        element.status = TrafficLightElement.SOLID_ON
        element.confidence = 1.0
        group.elements = [element]

        groups.append(group)

    msg.traffic_light_groups = groups
    return msg
