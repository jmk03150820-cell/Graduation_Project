import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fake_vehicle.vehicle_simulator import VehicleSimulator


class FakeVehicleNode(Node):
    """Simulates the vehicle's physical response to vehicle_interface_node's
    /vehicle/command, so the pipeline downstream of r2lp1_planning_node can
    be exercised before a real vehicle is connected. Stands in for both the
    real vehicle and (until it exists) vehicle_interface_node's hardware
    feedback path - see README for the assumed /vehicle/command schema.

    On missing/stale command data, targets decelerate to a full stop
    (fail-safe), matching the safety-default convention used elsewhere in
    this pipeline (cits_adapter, r2lp1_planning_node).
    """

    def __init__(self):
        super().__init__('fake_vehicle_node')

        self._declare_params()
        p = self._read_params()
        self._p = p

        self._sim = VehicleSimulator(
            max_accel_mps2=p['max_accel_mps2'],
            max_decel_mps2=p['max_decel_mps2'],
            max_steer_rate_deg_s=p['max_steer_rate_deg_s'],
        )
        self._last_command = None
        self._last_received_time = None
        self._last_step_time = time.time()

        self.create_subscription(String, p['command_topic'], self._on_command, 10)
        self.pub_status = self.create_publisher(String, p['status_topic'], 10)

        self.create_timer(1.0 / p['publish_rate_hz'], self._on_timer)

        self.get_logger().info(
            f"fake_vehicle_node started: {p['command_topic']} -> {p['status_topic']}")

    def _declare_params(self):
        defaults = {
            'command_topic': '/vehicle/command',
            'status_topic': '/vehicle/raw_status',
            'publish_rate_hz': 20.0,
            'command_stale_timeout_s': 1.0,
            'max_accel_mps2': 2.0,
            'max_decel_mps2': 3.0,
            'max_steer_rate_deg_s': 45.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'command_topic', 'status_topic', 'publish_rate_hz',
            'command_stale_timeout_s', 'max_accel_mps2', 'max_decel_mps2',
            'max_steer_rate_deg_s',
        ]
        return {name: self.get_parameter(name).value for name in names}

    def _on_command(self, msg):
        try:
            data = json.loads(msg.data)
            self._last_command = (
                float(data['target_speed_kmh']), float(data['target_steer_deg']))
            self._last_received_time = time.time()
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f'Bad /vehicle/command payload: {e}')

    def _on_timer(self):
        p = self._p
        now = time.time()
        dt = now - self._last_step_time
        self._last_step_time = now

        if (self._last_received_time is not None
                and (now - self._last_received_time) <= p['command_stale_timeout_s']):
            target_speed_kmh, target_steer_deg = self._last_command
        else:
            target_speed_kmh, target_steer_deg = 0.0, 0.0

        speed_kmh, steer_deg = self._sim.step(dt, target_speed_kmh, target_steer_deg)

        status = String()
        status.data = json.dumps({
            'speed_kmh': speed_kmh,
            'steer_deg': steer_deg,
            'timestamp': now,
        })
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
