import json
import time

import rclpy
from autoware_perception_msgs.msg import DetectedObjects, TrafficLightGroupArray
from rclpy.node import Node
from std_msgs.msg import String

from module_integrate.chameleon_builder import (
    extract_closest_hazard,
    extract_worst_traffic_light_state,
)


class ModuleIntegrateNode(Node):
    """Master hub (M): buffers cits_integration's two Autoware-native
    perception outputs (fused RSU+DENM objects, fused SPAT traffic
    signals), time-syncs them, and republishes a single master JSON on
    /integrate/chameleon_in for r2lp1_planning_node.

    If both inputs are stale at once, stops publishing entirely instead of
    emitting an artificially "fresh" but empty message - this lets
    r2lp1_planning_node's own chameleon_in staleness check trip a safe
    stop rather than being masked by a hollow heartbeat.
    """

    def __init__(self):
        super().__init__('module_integrate_node')

        self._declare_params()
        p = self._read_params()
        self._p = p

        self._last_objects = None
        self._last_objects_time = None
        self._last_signals = None
        self._last_signals_time = None
        self._max_sync_error_ms = 0.0

        self.create_subscription(
            DetectedObjects, p['input_objects_topic'], self._on_objects, 10)
        self.create_subscription(
            TrafficLightGroupArray, p['input_traffic_signals_topic'], self._on_signals, 10)

        self.pub_chameleon = self.create_publisher(String, p['output_topic'], 10)
        self.pub_log = self.create_publisher(String, p['log_topic'], 10)

        self.create_timer(1.0 / p['output_rate_hz'], self._on_timer)

        self.get_logger().info(
            f"module_integrate_node started: "
            f"{p['input_objects_topic']}, {p['input_traffic_signals_topic']} "
            f"-> {p['output_topic']}"
        )

    def _declare_params(self):
        defaults = {
            'input_objects_topic': '/perception/object_recognition/detection/objects',
            'input_traffic_signals_topic': '/perception/traffic_light_recognition/traffic_signals',
            'output_topic': '/integrate/chameleon_in',
            'log_topic': '/system/log',
            'output_rate_hz': 10.0,
            'objects_stale_timeout_s': 1.0,
            'signals_stale_timeout_s': 1.0,
            'ego_x': 0.0,
            'ego_y': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'input_objects_topic', 'input_traffic_signals_topic', 'output_topic',
            'log_topic', 'output_rate_hz', 'objects_stale_timeout_s',
            'signals_stale_timeout_s', 'ego_x', 'ego_y',
        ]
        return {name: self.get_parameter(name).value for name in names}

    def _on_objects(self, msg):
        self._last_objects = msg
        self._last_objects_time = time.time()

    def _on_signals(self, msg):
        self._last_signals = msg
        self._last_signals_time = time.time()

    def _on_timer(self):
        p = self._p
        now = time.time()

        objects_fresh = (
            self._last_objects_time is not None
            and (now - self._last_objects_time) <= p['objects_stale_timeout_s'])
        signals_fresh = (
            self._last_signals_time is not None
            and (now - self._last_signals_time) <= p['signals_stale_timeout_s'])

        if not objects_fresh and not signals_fresh:
            return

        sync_error_ms = None
        if objects_fresh and signals_fresh:
            sync_error_ms = abs(self._last_objects_time - self._last_signals_time) * 1000.0
            self._max_sync_error_ms = max(self._max_sync_error_ms, sync_error_ms)

        payload = {
            'traffic_light': {
                'state': (extract_worst_traffic_light_state(self._last_signals)
                          if signals_fresh else 'UNKNOWN'),
            },
            'timestamp': now,
        }
        if objects_fresh:
            hazard = extract_closest_hazard(self._last_objects, p['ego_x'], p['ego_y'])
            if hazard is not None:
                payload['hazard'] = hazard

        msg = String()
        msg.data = json.dumps(payload)
        self.pub_chameleon.publish(msg)

        self._publish_log(sync_error_ms)

    def _publish_log(self, sync_error_ms):
        msg = String()
        msg.data = json.dumps({
            'node': 'module_integrate_node',
            'event': 'sync',
            'sync_error_ms': sync_error_ms,
            'max_sync_error_ms': self._max_sync_error_ms,
            'timestamp': time.time(),
        })
        self.pub_log.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModuleIntegrateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
