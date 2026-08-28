"""ros2_convert 왕복(dc -> ROS msg -> dc) 계약 시험. ROS 2 Humble + rclpy +
빌드된 avva_interfaces 패키지 필요 (WSL): source /opt/ros/humble/setup.bash &&
source ~/cits_test_ws/install/setup.bash 후 실행.
"""
from __future__ import annotations

import avva_interfaces.msg as ros_msgs
import avva_phase1 as m
from simulation_layer import hashing
from simulation_layer import ros2_convert as conv


def _roundtrip(dc_obj):
    ros_obj = conv.to_ros_msg(dc_obj, ros_msgs)
    return conv.from_ros_msg(ros_obj, type(dc_obj))


def test_roundtrip_ego_control_command():
    """왕복 후에도 §4.6 canonical hash가 그대로면 필드 손실/왜곡이 없다는 뜻."""
    ctrl = m.NeutralControl(
        control_mode=m.ControlMode.DIRECT_ACTUATION,
        valid_fields_mask=(1 << 0) | (1 << 5) | (1 << 6),
        steering_tire_angle_rad=0.12, steering_tire_rotation_rate_rad_s=0.0,
        velocity_mps=0.0, acceleration_mps2=0.0, jerk_mps3=0.0,
        throttle=0.4, brake=0.0, gear=m.Gear.DRIVE, hand_brake=False)
    cmd = m.EgoControlCommand(
        header=m.CommonHeader(
            schema_major=1, schema_minor=0, run_id=b"\x01" * 16, run_epoch=1,
            scope_kind=m.ScopeKind.EGO, sim_id=m.OptionalBoundedId(True, "carla_0"),
            ego_id=m.OptionalBoundedId(True, "ego_0"), producer_id=m.ComponentId.EGO_ADAPTER,
            producer_instance_id=b"\x02" * 16, event_seq=1,
            correlation_id=m.OptionalUuid128(True, b"\x03" * 16),
            sim_time_ns=m.OptionalUint64(True, 1_000_000), wall_time_unix_ns=123,
            payload_hash=b"\x00" * 32),
        based_on_tick_id=0, target_tick_id=1, source_stack=m.SourceStack.MODULE_CHAIN,
        control_time_sim_ns=0, command_status=m.CommandStatus.COMMAND_OK,
        control=m.OptionalNeutralControl(True, ctrl), failure_reason=m.UNKNOWN_REASON)
    cmd.header.payload_hash = hashing.ego_control_command_hash(cmd)

    back = _roundtrip(cmd)
    assert back == cmd, "EgoControlCommand 왕복 후 필드가 달라짐"
    assert hashing.ego_control_command_hash(back) == cmd.header.payload_hash


def test_roundtrip_optional_none_value():
    """HOLD처럼 control이 없는(has_value=False, value=None) 경우도 왕복돼야 함."""
    item = m.NpcControlItem("npc_1", m.CommandAction.HOLD, m.ItemStatus.ITEM_OK,
                            m.OptionalNeutralControl(False, None), m.UNKNOWN_REASON)
    back = _roundtrip(item)
    assert back.action == m.CommandAction.HOLD
    assert back.control.has_value is False  # None -> 기본 NeutralControl 경유해도 has_value는 보존


if __name__ == "__main__":
    test_roundtrip_ego_control_command()
    test_roundtrip_optional_none_value()
    print("ros2_convert roundtrip contract test OK")
