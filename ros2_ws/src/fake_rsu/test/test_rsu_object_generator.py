"""Pure-Python tests for rsu_object_generator; no ROS install required.

Run standalone with:  python3 -m pytest test/test_rsu_object_generator.py
"""

from fake_rsu.rsu_object_generator import CAR, PERSON, generate_objects


def test_generates_one_pedestrian_and_one_car():
    objs = generate_objects(now=0.0)
    types = sorted(o['object_type'] for o in objs)
    assert types == sorted([PERSON, CAR])


def test_object_ids_are_distinct_and_configurable():
    objs = generate_objects(now=0.0, pedestrian_id=10, car_id=20)
    ids = {o['object_id'] for o in objs}
    assert ids == {10, 20}


def test_positions_change_over_time():
    a = generate_objects(now=0.0)
    b = generate_objects(now=3.0)
    a_positions = [(o['position_x'], o['position_y']) for o in a]
    b_positions = [(o['position_x'], o['position_y']) for o in b]
    assert a_positions != b_positions


def test_confidence_in_valid_range():
    for obj in generate_objects(now=5.0):
        assert 0.0 <= obj['confidence'] <= 1.0


def test_timestamp_is_milliseconds_and_matches_now():
    objs = generate_objects(now=1700000000.5)
    for obj in objs:
        assert obj['timestamp'] == 1700000000500
