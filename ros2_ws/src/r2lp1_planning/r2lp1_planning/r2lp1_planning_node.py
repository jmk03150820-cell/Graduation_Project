import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from r2lp1_planning.decision_logic import decide, parse_chameleon_json


class R2lp1PlanningNode(Node):
    """Phase 1 (if-else) stand-in for the reinforcement-learning planner.

    Subscribes to module_integrate_node's synced /integrate/chameleon_in
    JSON and publishes a target Twist on /autoware/control at a fixed
    rate (so control keeps flowing even if chameleon_in goes stale -
    stale/missing data always decides to a safe stop). Also reports each
    decision to /system/log for system_logger_node, since receipt-to-
    decision latency is this node's key metric.
    """

    def __init__(self):
        super().__init__('r2lp1_planning_node')

        self._declare_params()
        p = self._read_params()
        self._p = p

        self._last_data = None
        self._last_received_time = None

        self.create_subscription(String, p['input_topic'], self._on_chameleon_in, 10)
        self.pub_control = self.create_publisher(Twist, p['output_topic'], 10)
        self.pub_log = self.create_publisher(String, p['log_topic'], 10)

        self.create_timer(1.0 / p['decision_rate_hz'], self._on_decision_timer)

        self.get_logger().info(
            f"r2lp1_planning_node started: {p['input_topic']} -> {p['output_topic']}")

    def _declare_params(self):
        defaults = {
            'input_topic': '/integrate/chameleon_in',
            'output_topic': '/autoware/control',
            'log_topic': '/system/log',
            'decision_rate_hz': 10.0,
            'data_stale_timeout_s': 1.0,
            'target_speed_mps': 5.0,
            'min_speed_mps': 1.0,
            'emergency_stop_distance_m': 5.0,
            'slow_down_distance_m': 15.0,
            'steering_gain': 0.02,
            'max_steer_angular_z': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'input_topic', 'output_topic', 'log_topic', 'decision_rate_hz',
            'data_stale_timeout_s', 'target_speed_mps', 'min_speed_mps',
            'emergency_stop_distance_m', 'slow_down_distance_m',
            'steering_gain', 'max_steer_angular_z',
        ]
        return {name: self.get_parameter(name).value for name in names}

    def _on_chameleon_in(self, msg):
        try:
            self._last_data = parse_chameleon_json(msg.data)
            self._last_received_time = time.time()
        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().warn(f'Bad chameleon_in JSON: {e}')
            self._publish_log('parse_error', None, str(e))

    def _on_decision_timer(self):
        p = self._p
        now = time.time()

        data = None
        if (self._last_received_time is not None
                and (now - self._last_received_time) <= p['data_stale_timeout_s']):
            data = self._last_data

        decision = decide(data, p)

        twist = Twist()
        twist.linear.x = decision.linear_x
        twist.angular.z = decision.angular_z
        self.pub_control.publish(twist)

        latency_ms = (
            (now - self._last_received_time) * 1000.0
            if self._last_received_time is not None else None)
        self._publish_log(decision.event, latency_ms, decision.error)

    def _publish_log(self, event, latency_ms, error):
        msg = String()
        msg.data = json.dumps({
            'node': 'r2lp1_planning_node',
            'event': event,
            'latency_ms': latency_ms,
            'error': error,
            'timestamp': time.time(),
        })
        self.pub_log.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = R2lp1PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
