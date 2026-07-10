import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fake_rsu.message_generator import RsuSimulator


class FakeRsuNode(Node):
    """Publishes simplified fake C-ITS messages (CAM/DENM/SPAT/MAP) as JSON
    strings, standing in for a real roadside unit until an ASN.1/UPER V2X
    stack is wired up. Downstream, the Autoware Perception/Planning Adapter
    subscribes to these topics and converts them into Autoware-native inputs.
    """

    def __init__(self):
        super().__init__('fake_rsu_node')

        self._declare_params()
        p = self._read_params()

        self.sim = RsuSimulator(
            station_id=p['station_id'],
            intersection_id=p['intersection_id'],
            ref_lat=p['ref_lat'],
            ref_lon=p['ref_lon'],
            lane_length_m=p['lane_length_m'],
            vehicle_speed_mps=p['vehicle_speed_mps'],
            vehicle_length_m=p['vehicle_length_m'],
            vehicle_width_m=p['vehicle_width_m'],
            green_s=p['green_s'],
            yellow_s=p['yellow_s'],
            all_red_s=p['all_red_s'],
            denm_interval_s=p['denm_interval_s'],
            denm_duration_s=p['denm_duration_s'],
            denm_cause=p['denm_cause'],
            denm_approach=p['denm_approach'],
            denm_distance_m=p['denm_distance_m'],
            cam_approach=p['cam_approach'],
            cam_station_id=p['cam_station_id'],
        )

        prefix = p['topic_prefix']
        self.pub_cam = self.create_publisher(String, f'{prefix}/cam', 10)
        self.pub_denm = self.create_publisher(String, f'{prefix}/denm', 10)
        self.pub_spat = self.create_publisher(String, f'{prefix}/spat', 10)
        self.pub_map = self.create_publisher(String, f'{prefix}/map', 1)

        self._last_denm_active = False

        self.create_timer(1.0 / p['cam_rate_hz'], self._on_cam_timer)
        self.create_timer(1.0 / p['spat_rate_hz'], self._on_spat_timer)
        self.create_timer(1.0 / p['map_rate_hz'], self._on_map_timer)
        self.create_timer(1.0 / p['denm_check_rate_hz'], self._on_denm_timer)

        self.get_logger().info(
            f"fake_rsu_node started: intersection_id={p['intersection_id']} "
            f"topics=[{prefix}/cam, {prefix}/denm, {prefix}/spat, {prefix}/map]"
        )

    def _declare_params(self):
        defaults = {
            'topic_prefix': 'fake_rsu',
            'station_id': 9000,
            'intersection_id': 1,
            'ref_lat': 37.5665,
            'ref_lon': 126.9780,
            'lane_length_m': 60.0,
            'vehicle_speed_mps': 8.0,
            'vehicle_length_m': 4.5,
            'vehicle_width_m': 1.9,
            'green_s': 15.0,
            'yellow_s': 3.0,
            'all_red_s': 2.0,
            'denm_interval_s': 30.0,
            'denm_duration_s': 10.0,
            'denm_cause': 'roadworks',
            'denm_approach': 'north',
            'denm_distance_m': 25.0,
            'cam_approach': 'south',
            'cam_station_id': 42,
            'cam_rate_hz': 10.0,
            'spat_rate_hz': 2.0,
            'map_rate_hz': 0.2,
            'denm_check_rate_hz': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'topic_prefix', 'station_id', 'intersection_id', 'ref_lat', 'ref_lon',
            'lane_length_m', 'vehicle_speed_mps', 'vehicle_length_m', 'vehicle_width_m',
            'green_s', 'yellow_s', 'all_red_s', 'denm_interval_s', 'denm_duration_s',
            'denm_cause', 'denm_approach', 'denm_distance_m', 'cam_approach',
            'cam_station_id', 'cam_rate_hz', 'spat_rate_hz', 'map_rate_hz',
            'denm_check_rate_hz',
        ]
        return {name: self.get_parameter(name).value for name in names}

    def _publish(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)

    def _on_cam_timer(self):
        self._publish(self.pub_cam, self.sim.generate_cam(time.time()))

    def _on_spat_timer(self):
        self._publish(self.pub_spat, self.sim.generate_spat(time.time()))

    def _on_map_timer(self):
        self._publish(self.pub_map, self.sim.generate_map())

    def _on_denm_timer(self):
        denm = self.sim.generate_denm(time.time())
        if denm is not None:
            self._publish(self.pub_denm, denm)
            if not self._last_denm_active:
                self.get_logger().info(f"DENM event started: {denm['cause_code']} "
                                        f"(event_id={denm['event_id']})")
        elif self._last_denm_active:
            self.get_logger().info('DENM event cleared')
        self._last_denm_active = denm is not None


def main(args=None):
    rclpy.init(args=args)
    node = FakeRsuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
