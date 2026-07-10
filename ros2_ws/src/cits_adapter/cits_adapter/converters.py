"""Converts cits_rsu_msgs/DetectedObject (the predicted C-ITS RSU schema)
into autoware_perception_msgs/DetectedObject.

Field-name/enum correctness verified against the installed
ros-humble-autoware-perception-msgs via `ros2 interface show` (see
cits_integration/converters.py, which uses the same message types).
"""

from autoware_perception_msgs.msg import (
    DetectedObject as AutowareDetectedObject,
)
from autoware_perception_msgs.msg import (
    DetectedObjectKinematics,
    ObjectClassification,
    Shape,
)
from cits_rsu_msgs.msg import DetectedObject as RsuDetectedObject

from cits_adapter.geo import (
    bev_heading_to_map_yaw_rad,
    bev_to_map_xy,
    yaw_to_quaternion_xyzw,
)

# object_type code -> Autoware classification label. Only PERSON=1 was
# confirmed by the teammate; the rest are placeholder guesses.
_TYPE_TO_CLASSIFICATION = {
    RsuDetectedObject.UNKNOWN: ObjectClassification.UNKNOWN,
    RsuDetectedObject.PERSON: ObjectClassification.PEDESTRIAN,
    RsuDetectedObject.CAR: ObjectClassification.CAR,
    RsuDetectedObject.BICYCLE: ObjectClassification.BICYCLE,
    RsuDetectedObject.MOTORCYCLE: ObjectClassification.MOTORCYCLE,
    RsuDetectedObject.TRUCK: ObjectClassification.TRUCK,
    RsuDetectedObject.BUS: ObjectClassification.BUS,
}

# Default bounding-box dimensions (length, width, height in meters) per
# type, since the RSU schema doesn't include object size. Rough real-world
# averages; tune once real detections are available.
_DEFAULT_DIMENSIONS = {
    ObjectClassification.UNKNOWN: (1.0, 1.0, 1.0),
    ObjectClassification.PEDESTRIAN: (0.6, 0.6, 1.7),
    ObjectClassification.CAR: (4.5, 1.9, 1.8),
    ObjectClassification.BICYCLE: (1.8, 0.6, 1.5),
    ObjectClassification.MOTORCYCLE: (2.0, 0.8, 1.5),
    ObjectClassification.TRUCK: (7.0, 2.5, 3.0),
    ObjectClassification.BUS: (10.0, 2.5, 3.2),
}


def rsu_object_to_detected_object(rsu_obj, rsu_x, rsu_y, rsu_yaw_deg):
    """rsu_obj: cits_rsu_msgs.msg.DetectedObject
    (rsu_x, rsu_y, rsu_yaw_deg): the RSU's own pose in the map frame."""
    map_x, map_y = bev_to_map_xy(
        rsu_obj.position_x, rsu_obj.position_y, rsu_x, rsu_y, rsu_yaw_deg)
    yaw = bev_heading_to_map_yaw_rad(rsu_obj.heading, rsu_yaw_deg)
    qx, qy, qz, qw = yaw_to_quaternion_xyzw(yaw)

    label = _TYPE_TO_CLASSIFICATION.get(rsu_obj.object_type, ObjectClassification.UNKNOWN)
    length, width, height = _DEFAULT_DIMENSIONS[label]

    obj = AutowareDetectedObject()
    obj.existence_probability = rsu_obj.confidence

    classification = ObjectClassification()
    classification.label = label
    classification.probability = 1.0
    obj.classification = [classification]

    k = obj.kinematics
    k.pose_with_covariance.pose.position.x = float(map_x)
    k.pose_with_covariance.pose.position.y = float(map_y)
    k.pose_with_covariance.pose.position.z = 0.0
    k.pose_with_covariance.pose.orientation.x = qx
    k.pose_with_covariance.pose.orientation.y = qy
    k.pose_with_covariance.pose.orientation.z = qz
    k.pose_with_covariance.pose.orientation.w = qw
    k.has_position_covariance = False
    k.orientation_availability = DetectedObjectKinematics.AVAILABLE
    k.twist_with_covariance.twist.linear.x = float(rsu_obj.velocity)
    k.has_twist = True
    k.has_twist_covariance = False

    obj.shape.type = Shape.BOUNDING_BOX
    obj.shape.dimensions.x = length
    obj.shape.dimensions.y = width
    obj.shape.dimensions.z = height

    return obj
