import rclpy
from autoware_perception_msgs.msg import DetectedObjects, TrafficLightGroupArray
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from perception_markers.marker_builder import (
    build_lane_vehicle_markers,
    build_object_markers,
    build_traffic_light_markers,
)


class PerceptionMarkersNode(Node):
    """Converts Autoware's custom perception messages (DetectedObjects,
    TrafficLightGroupArray) into visualization_msgs/MarkerArray, since
    Foxglove (and RViz) don't understand Autoware's message types natively
    and would otherwise only show them as raw/unrendered data in a 3D panel.
    """

    def __init__(self):
        super().__init__('perception_markers_node')

        self._declare_params()
        p = self._read_params()
        self._p = p

        self.create_subscription(
            DetectedObjects, p['input_objects_topic'], self._on_objects, 10)
        self.create_subscription(
            TrafficLightGroupArray, p['input_traffic_signals_topic'], self._on_signals, 10)

        self.pub_object_markers = self.create_publisher(MarkerArray, p['object_markers_topic'], 10)
        self.pub_signal_markers = self.create_publisher(MarkerArray, p['signal_markers_topic'], 10)
        self.pub_lane_vehicle_markers = self.create_publisher(
            MarkerArray, p['lane_vehicle_markers_topic'], 10)

        self.get_logger().info(
            f"perception_markers_node started: "
            f"{p['input_objects_topic']} -> {p['object_markers_topic']}, "
            f"{p['input_traffic_signals_topic']} -> {p['signal_markers_topic']}"
        )

    def _declare_params(self):
        defaults = {
            'input_objects_topic': '/perception/object_recognition/detection/objects',
            'input_traffic_signals_topic': '/perception/traffic_light_recognition/traffic_signals',
            'object_markers_topic': '/perception/object_recognition/detection/objects/markers',
            'signal_markers_topic': '/perception/traffic_light_recognition/traffic_signals/markers',
            'marker_lifetime_s': 0.5,
            'traffic_light_frame_id': 'map',
            'traffic_light_x': 10.0,
            'traffic_light_y': 0.0,
            'traffic_light_z': 3.0,
            'traffic_light_lane_spacing_m': 4.0,
            'lane_vehicle_markers_topic': '/perception/traffic_light_recognition/lane_vehicles',
            'lane_vehicle_approach_distance_m': 6.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'input_objects_topic', 'input_traffic_signals_topic',
            'object_markers_topic', 'signal_markers_topic', 'marker_lifetime_s',
            'traffic_light_frame_id', 'traffic_light_x', 'traffic_light_y', 'traffic_light_z',
            'traffic_light_lane_spacing_m', 'lane_vehicle_markers_topic',
            'lane_vehicle_approach_distance_m',
        ]
        return {name: self.get_parameter(name).value for name in names}

    def _on_objects(self, msg):
        markers = build_object_markers(msg, lifetime_s=self._p['marker_lifetime_s'])
        self.pub_object_markers.publish(markers)

    def _on_signals(self, msg):
        p = self._p
        position = (p['traffic_light_x'], p['traffic_light_y'], p['traffic_light_z'])

        markers = build_traffic_light_markers(
            msg, p['traffic_light_frame_id'], position,
            lifetime_s=p['marker_lifetime_s'],
            lane_spacing_m=p['traffic_light_lane_spacing_m'])
        self.pub_signal_markers.publish(markers)

        lane_vehicle_markers = build_lane_vehicle_markers(
            msg, p['traffic_light_frame_id'], position,
            lane_spacing_m=p['traffic_light_lane_spacing_m'],
            approach_distance_m=p['lane_vehicle_approach_distance_m'],
            lifetime_s=p['marker_lifetime_s'])
        self.pub_lane_vehicle_markers.publish(lane_vehicle_markers)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
