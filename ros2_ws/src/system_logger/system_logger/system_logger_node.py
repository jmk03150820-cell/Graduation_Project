import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from system_logger.log_record import LogStats, format_log_line, parse_log_json


class SystemLoggerNode(Node):
    """Terminal sink for /system/log: every other node in the pipeline
    (module_integrate_node, r2lp1_planning_node, ...) publishes its own
    JSON status/metric line here instead of managing its own log file.

    Parses each line, echoes a human-readable version to the console (and
    optionally appends it to a file), and periodically logs a rolling
    summary (per-node event counts, worst-case *_ms fields seen) so
    pipeline health can be read off one topic instead of grepping every
    node's console output.
    """

    def __init__(self):
        super().__init__('system_logger_node')

        self._declare_params()
        p = self._read_params()
        self._p = p

        self._stats = LogStats()
        self._file = open(p['log_file_path'], 'a', encoding='utf-8') if p['log_file_path'] else None

        self.create_subscription(String, p['log_topic'], self._on_log, 50)
        self.create_timer(p['summary_interval_s'], self._on_summary_timer)

        self.get_logger().info(
            f"system_logger_node started: listening on {p['log_topic']}")

    def _declare_params(self):
        defaults = {
            'log_topic': '/system/log',
            'summary_interval_s': 30.0,
            'log_file_path': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_params(self):
        names = ['log_topic', 'summary_interval_s', 'log_file_path']
        return {name: self.get_parameter(name).value for name in names}

    def _on_log(self, msg):
        try:
            record = parse_log_json(msg.data)
        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().warn(f'Bad /system/log JSON: {e}')
            return

        line = format_log_line(record)
        self.get_logger().info(line)
        self._stats.record(record)

        if self._file is not None:
            self._file.write(line + '\n')
            self._file.flush()

    def _on_summary_timer(self):
        self.get_logger().info(self._stats.summary())

    def destroy_node(self):
        if self._file is not None:
            self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SystemLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
