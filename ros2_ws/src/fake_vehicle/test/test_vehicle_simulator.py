"""Pure-Python tests for vehicle_simulator.py; no ROS install required.

Run standalone with:  python3 -m pytest test/test_vehicle_simulator.py
"""

import math

from fake_vehicle.vehicle_simulator import VehicleSimulator


def test_starts_at_rest():
    sim = VehicleSimulator()
    assert sim.speed_kmh == 0.0
    assert sim.steer_deg == 0.0


def test_accel_is_rate_limited_not_instant():
    sim = VehicleSimulator(max_accel_mps2=2.0)
    speed, _ = sim.step(dt_s=1.0, target_speed_kmh=100.0, target_steer_deg=0.0)
    assert math.isclose(speed, 2.0 * 3.6)
    assert speed < 100.0


def test_reaches_target_speed_without_overshoot():
    sim = VehicleSimulator(max_accel_mps2=2.0)
    for _ in range(100):
        speed, _ = sim.step(dt_s=1.0, target_speed_kmh=20.0, target_steer_deg=0.0)
    assert math.isclose(speed, 20.0)


def test_decel_uses_separate_limit_from_accel():
    sim = VehicleSimulator(max_accel_mps2=1000.0, max_decel_mps2=1.0)
    sim.step(dt_s=1.0, target_speed_kmh=50.0, target_steer_deg=0.0)  # snap to target (huge accel)
    speed, _ = sim.step(dt_s=1.0, target_speed_kmh=0.0, target_steer_deg=0.0)
    assert math.isclose(speed, 50.0 - 1.0 * 3.6)


def test_speed_never_goes_negative():
    sim = VehicleSimulator(max_decel_mps2=100.0)
    sim.step(dt_s=1.0, target_speed_kmh=10.0, target_steer_deg=0.0)
    speed, _ = sim.step(dt_s=1.0, target_speed_kmh=-50.0, target_steer_deg=0.0)
    assert speed == 0.0


def test_steer_is_rate_limited_and_reaches_target():
    sim = VehicleSimulator(max_steer_rate_deg_s=10.0)
    _, steer = sim.step(dt_s=1.0, target_speed_kmh=0.0, target_steer_deg=30.0)
    assert math.isclose(steer, 10.0)
    for _ in range(10):
        _, steer = sim.step(dt_s=1.0, target_speed_kmh=0.0, target_steer_deg=30.0)
    assert math.isclose(steer, 30.0)


def test_steer_tracks_negative_targets_too():
    sim = VehicleSimulator(max_steer_rate_deg_s=90.0)
    _, steer = sim.step(dt_s=1.0, target_speed_kmh=0.0, target_steer_deg=-20.0)
    assert math.isclose(steer, -20.0)
