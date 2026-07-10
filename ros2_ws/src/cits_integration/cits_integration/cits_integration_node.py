import json
import time

import rclpy
from autoware_perception_msgs.msg import DetectedObjects, TrafficLightGroupArray
from rclpy.node import Node
from std_msgs.msg import String

from cits_integration.converters import (
    build_detected_objects,
    denm_to_detected_object,
    spat_to_traffic_light_group_array,
)


class CitsIntegrationNode(Node):
    """Merges cits_adapter's RSU-detected-object feed with fake_rsu's
    SPAT/DENM JSON into the final Autoware Universe perception topics.

    cits_adapter now owns the C-ITS RSU object -> Autoware DetectedObject
    conversion (it sits closer to the source and knows the RSU's own map
    pose); this node just combines that with the DENM-derived hazard object
    and republishes SPAT as TrafficLightGroupArray. A perception_ai_node
    input can be added as a further subscriber once its output contract is
    agreed with the teammate building it.
    """

    def __init__(self):
        super().__init__('cits_integration_node')

        self._declare_params()
        p = self._read_params()
        self._p = p

        self._last_adapter_objects = []
        self._last_adapter_time = None
        self._last_denm = None
        self._last_denm_time = None

        self.create_subscription(
            DetectedObjects, p['input_adapter_objects_topic'], self._on_adapter_objects, 10)
        self.create_subscription(String, p['input_spat_topic'], self._on_spat, 10)
        self.create_subscription(String, p['input_denm_topic'], self._on_denm, 10)

        self.pub_objects = self.create_publisher(
            DetectedObjects, p['output_objects_topic'], 10)
        self.pub_traffic_signals = self.create_publisher(
            TrafficLightGroupArray, p['output_traffic_signals_topic'], 10)

        self.create_timer(1.0 / p['object_publish_rate_hz'], self._on_publish_objects_timer)

        self.get_logger().info(
            f"cits_integration_node started: "
            f"{p['input_adapter_objects_topic']}, {p['input_spat_topic']}, {p['input_denm_topic']} "
            f"-> {p['output_objects_topic']}, {p['output_traffic_signals_topic']}"
        )

    def _declare_params(self):
        defaults = {
            'input_adapter_objects_topic': 'cits_adapter/objects',
            'input_spat_topic': 'fake_rsu/spat',
            'input_denm_topic': 'fake_rsu/denm',
            'output_objects_topic': '/perception/object_recognition/detection/objects',
            'output_traffic_signals_topic': '/perception/traffic_light_recognition/traffic_signals',
            'map_origin_lat': 37.5665,
            'map_origin_lon': 126.9780,
            'output_frame_id': 'map',
            'object_publish_rate_hz': 10.0,
            'adapter_stale_timeout_s': 2.0,
            'denm_stale_timeout_s': 2.0,
            'hazard_size_m': 2.0,
            'hazard_height_m': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'input_adapter_objects_topic', 'input_spat_topic', 'input_denm_topic',
            'output_objects_topic', 'output_traffic_signals_topic',
            'map_origin_lat', 'map_origin_lon', 'output_frame_id',
            'object_publish_rate_hz', 'adapter_stale_timeout_s', 'denm_stale_timeout_s',
            'hazard_size_m', 'hazard_height_m',
        ]
        return {name: self.get_parameter(name).value for name in names}

    def _on_adapter_objects(self, msg):
        self._last_adapter_objects = list(msg.objects)
        self._last_adapter_time = time.time()

    def _on_denm(self, msg):
        try:
            self._last_denm = json.loads(msg.data)
            self._last_denm_time = time.time()
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Bad DENM JSON: {e}')

    def _on_spat(self, msg):
        try:
            spat = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Bad SPAT JSON: {e}')
            return

        traffic_signals = spat_to_traffic_light_group_array(spat, time.time())
        self.pub_traffic_signals.publish(traffic_signals)

    def _on_publish_objects_timer(self):
        p = self._p
        now = time.time()
        objects = []

        if (self._last_adapter_time is not None
                and (now - self._last_adapter_time) <= p['adapter_stale_timeout_s']):
            objects.extend(self._last_adapter_objects)

        if self._last_denm is not None and (now - self._last_denm_time) <= p['denm_stale_timeout_s']:
            objects.append(denm_to_detected_object(
                self._last_denm, p['map_origin_lat'], p['map_origin_lon'],
                hazard_size_m=p['hazard_size_m'], hazard_height_m=p['hazard_height_m']))

        detected_objects = build_detected_objects(objects, p['output_frame_id'], now)
        self.pub_objects.publish(detected_objects)


def main(args=None):
    rclpy.init(args=args)
    node = CitsIntegrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
