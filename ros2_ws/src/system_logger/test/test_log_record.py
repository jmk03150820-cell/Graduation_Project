"""Pure-Python tests for log_record.py; no ROS install required.

Run standalone with:  python3 -m pytest test/test_log_record.py -v
"""

import json

import pytest

from system_logger.log_record import LogStats, format_log_line, parse_log_json


def test_parse_log_json_roundtrip():
    payload = {'node': 'module_integrate_node', 'event': 'sync', 'timestamp': 1.0}
    assert parse_log_json(json.dumps(payload)) == payload


def test_parse_log_json_rejects_non_object():
    with pytest.raises(ValueError):
        parse_log_json(json.dumps([1, 2, 3]))


def test_parse_log_json_rejects_invalid_json():
    with pytest.raises(json.JSONDecodeError):
        parse_log_json('not json')


def test_format_log_line_includes_node_event_and_extras():
    record = {
        'node': 'r2lp1_planning_node',
        'event': 'cruise',
        'timestamp': 5.5,
        'latency_ms': 12.345,
        'error': None,
    }
    line = format_log_line(record)
    assert line.startswith('[5.500] r2lp1_planning_node cruise')
    assert 'latency_ms=12.345' in line
    assert 'error' not in line  # None-valued fields are dropped


def test_format_log_line_handles_missing_timestamp():
    assert format_log_line({'node': 'x', 'event': 'y'}) == '[?] x y'


def test_log_stats_counts_events_per_node():
    stats = LogStats()
    stats.record({'node': 'a', 'event': 'ok'})
    stats.record({'node': 'a', 'event': 'ok'})
    stats.record({'node': 'b', 'event': 'ok'})
    summary = stats.summary()
    assert 'total=3' in summary
    assert 'a.ok=2' in summary
    assert 'b.ok=1' in summary


def test_log_stats_tracks_max_ms_fields_per_node():
    stats = LogStats()
    stats.record({'node': 'a', 'event': 'e', 'latency_ms': 5.0})
    stats.record({'node': 'a', 'event': 'e', 'latency_ms': 9.5})
    stats.record({'node': 'a', 'event': 'e', 'latency_ms': 3.0})
    assert 'a.max_latency_ms=9.5' in stats.summary()


def test_log_stats_counts_errors():
    stats = LogStats()
    stats.record({'node': 'a', 'event': 'e', 'error': 'boom'})
    stats.record({'node': 'a', 'event': 'e', 'error': None})
    assert 'errors=1' in stats.summary()


def test_log_stats_empty_summary():
    assert LogStats().summary() == 'system_logger: no log records yet'
