import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

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

    TEMPORARY: also subscribes directly to r2lp1_planning_node's
    /autoware/control (Twist) and converts it to the same target
    speed/steer, used only when /vehicle/command hasn't been seen
    recently. This stands in for vehicle_interface_node's own conversion
    (m/s -> km/h, rad/s -> deg/s) so the pipeline is visibly drivable
    before that node exists - remove once vehicle_interface_node is real
    and reliably publishes /vehicle/command itself.

    Also broadcasts the simulator's dead-reckoned pose as a map -> ego TF
    and a MarkerArray, so the simulated motion (and stopping) is visible
    in RViz/Foxglove instead of only existing as JSON numbers.
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
        self._last_control = None
        self._last_control_time = None
        self._last_step_time = time.time()

        self.create_subscription(String, p['command_topic'], self._on_command, 10)
        self.create_subscription(Twist, p['autoware_control_topic'], self._on_autoware_control, 10)
        self.pub_status = self.create_publisher(String, p['status_topic'], 10)
        self.pub_marker = self.create_publisher(MarkerArray, p['ego_marker_topic'], 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / p['publish_rate_hz'], self._on_timer)

        self.get_logger().info(
            f"fake_vehicle_node started: {p['command_topic']} -> {p['status_topic']}")

    def _declare_params(self):
        defaults = {
            'command_topic': '/vehicle/command',
            'autoware_control_topic': '/autoware/control',
            'status_topic': '/vehicle/raw_status',
            'publish_rate_hz': 20.0,
            'command_stale_timeout_s': 1.0,
            'max_accel_mps2': 2.0,
            'max_decel_mps2': 3.0,
            'max_steer_rate_deg_s': 45.0,
            'ego_marker_topic': '/fake_vehicle/ego_marker',
            'map_frame_id': 'map',
            'ego_frame_id': 'base_link',
            'stopped_speed_threshold_kmh': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = [
            'command_topic', 'autoware_control_topic', 'status_topic', 'publish_rate_hz',
            'command_stale_timeout_s', 'max_accel_mps2', 'max_decel_mps2',
            'max_steer_rate_deg_s', 'ego_marker_topic', 'map_frame_id',
            'ego_frame_id', 'stopped_speed_threshold_kmh',
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

    def _on_autoware_control(self, msg):
        # m/s -> km/h, rad/s -> deg/s (see class docstring: temporary
        # stand-in for vehicle_interface_node's own conversion).
        self._last_control = (msg.linear.x * 3.6, math.degrees(msg.angular.z))
        self._last_control_time = time.time()

    def _on_timer(self):
        p = self._p
        now = time.time()
        dt = now - self._last_step_time
        self._last_step_time = now

        if (self._last_received_time is not None
                and (now - self._last_received_time) <= p['command_stale_timeout_s']):
            target_speed_kmh, target_steer_deg = self._last_command
        elif (self._last_control_time is not None
                and (now - self._last_control_time) <= p['command_stale_timeout_s']):
            target_speed_kmh, target_steer_deg = self._last_control
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

        self._broadcast_tf()
        self._publish_ego_marker(speed_kmh)

    def _broadcast_tf(self):
        p = self._p
        sim = self._sim
        qz, qw = math.sin(math.radians(sim.yaw_deg) / 2.0), math.cos(math.radians(sim.yaw_deg) / 2.0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = p['map_frame_id']
        t.child_frame_id = p['ego_frame_id']
        t.transform.translation.x = sim.x
        t.transform.translation.y = sim.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _publish_ego_marker(self, speed_kmh):
        p = self._p
        sim = self._sim
        qz, qw = math.sin(math.radians(sim.yaw_deg) / 2.0), math.cos(math.radians(sim.yaw_deg) / 2.0)

        marker = Marker()
        marker.header.frame_id = p['map_frame_id']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ego_vehicle'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = sim.x
        marker.pose.position.y = sim.y
        marker.pose.position.z = 0.9
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 4.5
        marker.scale.y = 1.9
        marker.scale.z = 1.8
        if speed_kmh < p['stopped_speed_threshold_kmh']:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.15, 0.15
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.15, 0.85, 0.25
        marker.color.a = 0.9

        self.pub_marker.publish(MarkerArray(markers=[marker]))


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
