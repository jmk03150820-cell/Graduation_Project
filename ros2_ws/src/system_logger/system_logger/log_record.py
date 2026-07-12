"""Pure-Python parsing/formatting for /system/log JSON entries published by
module_integrate_node and r2lp1_planning_node (see their READMEs for each
node's log payload fields). No rclpy dependency, so it's testable without a
ROS environment, matching the chameleon_builder.py / decision_logic.py
pattern used by the rest of the pipeline.
"""

import json


def parse_log_json(data):
    """Parse a /system/log String payload into a dict.

    Raises ValueError if the payload isn't valid JSON or isn't a JSON
    object (mirrors r2lp1_planning.decision_logic.parse_chameleon_json).
    """
    parsed = json.loads(data)
    if not isinstance(parsed, dict):
        raise ValueError(f'Expected a JSON object, got {type(parsed).__name__}')
    return parsed


def format_log_line(record):
    """Human-readable single-line rendering of a log record for console/file."""
    node = record.get('node', 'unknown')
    event = record.get('event', 'unknown')
    timestamp = record.get('timestamp')
    ts_str = f'{timestamp:.3f}' if isinstance(timestamp, (int, float)) else '?'

    extras = []
    for key, value in record.items():
        if key in ('node', 'event', 'timestamp') or value is None:
            continue
        if isinstance(value, float):
            value = round(value, 3)
        extras.append(f'{key}={value}')

    line = f'[{ts_str}] {node} {event}'
    if extras:
        line += ' ' + ' '.join(extras)
    return line


class LogStats:
    """Accumulates per-(node, event) counts and running per-node maxima for
    any numeric field ending in '_ms' (latency_ms, sync_error_ms, ...), so
    system_logger_node can periodically print a health summary instead of
    just echoing raw lines."""

    def __init__(self):
        self._total = 0
        self._error_count = 0
        self._event_counts = {}
        self._max_ms = {}

    def record(self, log_record):
        self._total += 1
        node = log_record.get('node', 'unknown')
        event = log_record.get('event', 'unknown')

        key = (node, event)
        self._event_counts[key] = self._event_counts.get(key, 0) + 1

        if log_record.get('error'):
            self._error_count += 1

        for field_name, value in log_record.items():
            if field_name.endswith('_ms') and isinstance(value, (int, float)):
                max_key = (node, field_name)
                current = self._max_ms.get(max_key)
                if current is None or value > current:
                    self._max_ms[max_key] = value

    def summary(self):
        if self._total == 0:
            return 'system_logger: no log records yet'

        parts = [f'total={self._total}', f'errors={self._error_count}']
        for (node, event), count in sorted(self._event_counts.items()):
            parts.append(f'{node}.{event}={count}')
        for (node, field_name), value in sorted(self._max_ms.items()):
            parts.append(f'{node}.max_{field_name}={round(value, 3)}')
        return 'system_logger summary: ' + ' '.join(parts)
