import time

import rclpy
from autoware_perception_msgs.msg import DetectedObjects, ObjectClassification
from cits_rsu_msgs.msg import DetectedObject, DetectedObjectArray
from rclpy.node import Node
from std_msgs.msg import Header

from cits_adapter.converters import rsu_object_to_detected_object

_RSU_TYPE_NAMES = {
    DetectedObject.UNKNOWN: 'UNKNOWN',
    DetectedObject.PERSON: 'PERSON',
    DetectedObject.CAR: 'CAR',
    DetectedObject.BICYCLE: 'BICYCLE',
    DetectedObject.MOTORCYCLE: 'MOTORCYCLE',
    DetectedObject.TRUCK: 'TRUCK',
    DetectedObject.BUS: 'BUS',
}

_AUTOWARE_LABEL_NAMES = {
    ObjectClassification.UNKNOWN: 'UNKNOWN',
    ObjectClassification.CAR: 'CAR',
    ObjectClassification.TRUCK: 'TRUCK',
    ObjectClassification.BUS: 'BUS',
    ObjectClassification.TRAILER: 'TRAILER',
    ObjectClassification.MOTORCYCLE: 'MOTORCYCLE',
    ObjectClassification.BICYCLE: 'BICYCLE',
    ObjectClassification.PEDESTRIAN: 'PEDESTRIAN',
}


class CitsAdapterNode(Node):
    """Adapter between the (real, or fake for now) C-ITS RSU detected-object
    feed and Autoware. Extracts only what's needed (position, heading,
    velocity, classification, confidence), applies the RSU's map-frame pose
    and a confidence/age filter, and republishes as
    autoware_perception_msgs/DetectedObjects for cits_integration to merge
    with the DENM-derived hazard objects.
    """

    def __init__(self):
        super().__init__('cits_adapter_node')

        self._declare_params()
        p = self._read_params()
        self._p = p

        self.create_subscription(
            DetectedObjectArray, p['input_topic'], self._on_objects, 10)
        self.pub_objects = self.create_publisher(
            DetectedObjects, p['output_topic'], 10)

        self.get_logger().info(
            f"cits_adapter_node started: {p['input_topic']} -> {p['output_topic']} "
            f"(rsu_pose=({p['rsu_x']}, {p['rsu_y']}, {p['rsu_yaw_deg']}deg))"
        )

    def _declare_params(self):
        defaults = {
            'input_topic': 'cits/rsu/detected_objects',
            'output_topic': 'cits_adapter/objects',
            'output_frame_id': 'map',
            # RSU's own pose in the map frame — placeholder until the real
            # RSU mounting position/orientation is surveyed.
            'rsu_x': 0.0,
            'rsu_y': 0.0,
            'rsu_yaw_deg': 0.0,
            'min_confidence': 0.0,
            'max_object_age_s': 2.0,
            'verbose_logging': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'input_topic', 'output_topic', 'output_frame_id',
            'rsu_x', 'rsu_y', 'rsu_yaw_deg',
            'min_confidence', 'max_object_age_s', 'verbose_logging',
        ]
        return {name: self.get_parameter(name).value for name in names}

    def _on_objects(self, msg):
        p = self._p
        now_ms = time.time() * 1000.0
        verbose = p['verbose_logging']

        if verbose and msg.objects:
            self.get_logger().info(
                f'--- RSU 원본 수신: {len(msg.objects)}개 객체 ({p["input_topic"]}) ---')

        objects = []
        for rsu_obj in msg.objects:
            type_name = _RSU_TYPE_NAMES.get(rsu_obj.object_type, str(rsu_obj.object_type))
            if verbose:
                self.get_logger().info(
                    f'  [원본] id={rsu_obj.object_id} type={type_name} '
                    f'pos=({rsu_obj.position_x:.2f}, {rsu_obj.position_y:.2f})[BEV,m] '
                    f'v={rsu_obj.velocity:.2f}m/s heading={rsu_obj.heading:.1f}deg '
                    f'confidence={rsu_obj.confidence:.2f}'
                )

            if rsu_obj.confidence < p['min_confidence']:
                if verbose:
                    self.get_logger().info(
                        f'  [제외] id={rsu_obj.object_id}: confidence '
                        f'{rsu_obj.confidence:.2f} < min_confidence {p["min_confidence"]:.2f}')
                continue
            age_s = (now_ms - rsu_obj.timestamp) / 1000.0
            if age_s > p['max_object_age_s']:
                if verbose:
                    self.get_logger().info(
                        f'  [제외] id={rsu_obj.object_id}: {age_s:.2f}s 지남 '
                        f'(max_object_age_s={p["max_object_age_s"]:.2f})')
                continue

            converted = rsu_object_to_detected_object(
                rsu_obj, p['rsu_x'], p['rsu_y'], p['rsu_yaw_deg'])
            objects.append(converted)

            if verbose:
                pos = converted.kinematics.pose_with_covariance.pose.position
                label_name = _AUTOWARE_LABEL_NAMES.get(
                    converted.classification[0].label, str(converted.classification[0].label))
                self.get_logger().info(
                    f'  [추출/가공] id={rsu_obj.object_id} -> label={label_name} '
                    f'pos=({pos.x:.2f}, {pos.y:.2f})[map,m] '
                    f'speed={converted.kinematics.twist_with_covariance.twist.linear.x:.2f}m/s '
                    f'existence_probability={converted.existence_probability:.2f} '
                    f'shape=({converted.shape.dimensions.x:.1f}x'
                    f'{converted.shape.dimensions.y:.1f}x{converted.shape.dimensions.z:.1f})'
                )

        out = DetectedObjects()
        out.header = Header()
        out.header.frame_id = p['output_frame_id']
        out.header.stamp = self.get_clock().now().to_msg()
        out.objects = objects
        self.pub_objects.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CitsAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
