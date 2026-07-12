"""Pure-Python tests for decision_logic.py; no ROS install required.

Run standalone with:  python3 -m pytest test/test_decision_logic.py
"""

import json

import pytest

from r2lp1_planning.decision_logic import decide, parse_chameleon_json

PARAMS = {
    'target_speed_mps': 5.0,
    'min_speed_mps': 1.0,
    'emergency_stop_distance_m': 5.0,
    'slow_down_distance_m': 15.0,
    'steering_gain': 0.02,
    'max_steer_angular_z': 0.5,
}


def test_missing_data_is_safe_stop():
    d = decide(None, PARAMS)
    assert d.linear_x == 0.0
    assert d.angular_z == 0.0
    assert d.error is not None


def test_red_light_stops_even_with_clear_road():
    data = {'traffic_light': {'state': 'RED'}}
    d = decide(data, PARAMS)
    assert d.linear_x == 0.0
    assert d.event.startswith('stop_traffic_light')


def test_green_light_with_no_hazard_cruises():
    data = {'traffic_light': {'state': 'GREEN'}}
    d = decide(data, PARAMS)
    assert d.linear_x == PARAMS['target_speed_mps']
    assert d.angular_z == 0.0
    assert d.event == 'cruise'


def test_hazard_inside_emergency_distance_stops():
    data = {'hazard': {'distance_m': 3.0, 'angle_deg': 0.0}}
    d = decide(data, PARAMS)
    assert d.linear_x == 0.0
    assert d.event == 'stop_hazard_too_close'


def test_hazard_in_slow_down_band_scales_speed_between_min_and_target():
    near = decide({'hazard': {'distance_m': 5.1, 'angle_deg': 0.0}}, PARAMS)
    far = decide({'hazard': {'distance_m': 14.9, 'angle_deg': 0.0}}, PARAMS)
    assert PARAMS['min_speed_mps'] <= near.linear_x < far.linear_x <= PARAMS['target_speed_mps']


def test_hazard_steer_is_clamped_and_away_from_angle():
    data = {'hazard': {'distance_m': 10.0, 'angle_deg': 1000.0}}
    d = decide(data, PARAMS)
    assert abs(d.angular_z) <= PARAMS['max_steer_angular_z']
    assert d.angular_z < 0  # steers away from a large positive (rightward) angle


def test_hazard_beyond_slow_down_distance_cruises():
    data = {'hazard': {'distance_m': 100.0, 'angle_deg': 0.0}}
    d = decide(data, PARAMS)
    assert d.linear_x == PARAMS['target_speed_mps']
    assert d.event == 'cruise'


def test_traffic_light_takes_priority_over_hazard():
    data = {
        'traffic_light': {'state': 'YELLOW'},
        'hazard': {'distance_m': 100.0, 'angle_deg': 0.0},
    }
    d = decide(data, PARAMS)
    assert d.linear_x == 0.0
    assert 'traffic_light' in d.event


def test_parse_chameleon_json_roundtrip():
    payload = {'traffic_light': {'state': 'GREEN'}}
    assert parse_chameleon_json(json.dumps(payload)) == payload


def test_parse_chameleon_json_rejects_non_object():
    with pytest.raises(ValueError):
        parse_chameleon_json(json.dumps([1, 2, 3]))
