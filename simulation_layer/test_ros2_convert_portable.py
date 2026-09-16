"""ros2_convert.py의 None-optional 처리 회귀 시험 — 진짜 avva_interfaces.msg(rclpy
필요, WSL 전용)는 안 쓰고 rosidl 고정길이 배열 기본값만 흉내 낸 가짜 ROS 모듈로
Windows에서도 그냥 돈다. test_ros2_convert.py(왕복 전체 계약)와는 별개로, 이 파일
하나는 "OptionalHash256(has_value=False, value=None)"처럼 bytes 타입 필드 자체가
None인 경우만 좁게 검증한다.

Runs with `python -m simulation_layer.test_ros2_convert_portable`.
"""
from __future__ import annotations

import simulation_layer  # noqa: F401  (interfaces path shim)
import avva_phase1 as m
from simulation_layer import ros2_convert as conv


class _FakeHash256Ros:
    def __init__(self) -> None:
        self.value = [0] * 32  # rosidl uint8[32] 기본 생성값 흉내


class _FakeRosModule:
    OptionalHash256 = _FakeHash256Ros


def test_none_bytes_field_leaves_ros_default_not_zero_length():
    """OptionalHash256.value(타입 bytes)가 None이면, 예전엔 _default_for('bytes')가
    길이 0인 b""를 넣어서 고정길이 uint8[32] ROS 필드가 깨졌다(코드 로직 버그 —
    실제 코드베이스는 항상 zero-fill을 써서 지금까지 발동은 안 했지만, 이 파일 자체
    docstring이 "value=None이 관례"라고 말하고 있어 잠재 위험이었음). 이제 None이면
    그 필드를 아예 안 건드리고 ROS가 이미 만들어둔(올바른 길이의) 기본값을 그대로
    둔다."""
    dc_obj = m.OptionalHash256(has_value=False, value=None)
    ros_obj = conv.to_ros_msg(dc_obj, _FakeRosModule)
    assert len(ros_obj.value) == 32, "bytes=None 처리로 고정길이 필드가 깨짐(길이 0이면 실패)"


TESTS = [
    (test_none_bytes_field_leaves_ros_default_not_zero_length,
     "bytes 타입 필드 자체가 None이면(OptionalHash256.value 등) ROS 기본 생성값을 안 건드려 "
     "고정길이 uint8[N] 필드가 길이 0으로 깨지지 않음"),
]

if __name__ == "__main__":
    for i, (fn, desc) in enumerate(TESTS, 1):
        fn()
        print(f"[{i}/{len(TESTS)}] PASS {fn.__name__}\n         검증: {desc}")
    print(f"\n전체 {len(TESTS)}개 ros2_convert 이식 가능 시험 PASS")
