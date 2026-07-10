"""Pure-Python tests for message_generator; no rclpy/ROS 2 install required.

Run standalone with:  python3 -m pytest test/test_message_generator.py
"""

from fake_rsu.message_generator import RsuSimulator


def make_sim(**overrides):
    kwargs = dict(
        ref_lat=37.5665, ref_lon=126.9780, lane_length_m=60.0,
        vehicle_speed_mps=10.0, green_s=10.0, yellow_s=2.0, all_red_s=1.0,
        denm_interval_s=20.0, denm_duration_s=5.0,
    )
    kwargs.update(overrides)
    return RsuSimulator(**kwargs)


def test_map_has_four_approaches_with_in_and_out_lanes():
    sim = make_sim()
    m = sim.generate_map()
    assert m['msg_type'] == 'MAP'
    assert len(m['lanes']) == 8
    approaches = {lane['approach'] for lane in m['lanes']}
    assert approaches == {'north', 'south', 'east', 'west'}


def test_map_is_cached_across_calls():
    sim = make_sim()
    assert sim.generate_map() is sim.generate_map()


def test_spat_reports_exactly_one_green_group_during_ns_phase():
    sim = make_sim()
    spat = sim.generate_spat(now=1.0)  # early in the NS-green phase
    states = {g['signal_group_id']: g['state'] for g in spat['signal_groups']}
    assert states[1] == 'green'
    assert states[2] == 'red'


def test_spat_cycles_to_ew_green_after_ns_phases_elapse():
    sim = make_sim()
    # NS green(10) + yellow(2) + all-red(1) = 13s elapsed -> EW green begins
    spat = sim.generate_spat(now=13.5)
    states = {g['signal_group_id']: g['state'] for g in spat['signal_groups']}
    assert states[2] == 'green'
    assert states[1] == 'red'


def test_cam_position_moves_between_two_calls():
    sim = make_sim()
    a = sim.generate_cam(now=0.0)
    b = sim.generate_cam(now=1.0)
    assert (a['position']['lat'], a['position']['lon']) != \
           (b['position']['lat'], b['position']['lon'])


def test_denm_is_none_outside_active_window():
    sim = make_sim()
    assert sim.generate_denm(now=10.0) is None  # after 5s duration, before next 20s cycle


def test_denm_is_present_during_active_window():
    sim = make_sim()
    denm = sim.generate_denm(now=1.0)
    assert denm is not None
    assert denm['msg_type'] == 'DENM'
    assert denm['event_id'] == 0

    denm2 = sim.generate_denm(now=21.0)  # second cycle
    assert denm2 is not None
    assert denm2['event_id'] == 1
