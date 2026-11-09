"""avva_phase1 dataclass <-> avva_interfaces ROS 2 message 양방향 변환.

gate.py는 여전히 avva_phase1 dataclass만 다룬다 (전제조건/큰 프레임 불변) —
이 모듈이 그 경계에서 ROS 2 wire 타입으로/에서 변환한다. avva_phase1.py와
avva_interfaces.msg의 struct는 같은 schema/avva_phase1.idl에서 1:1로 생성돼
클래스명·필드명·필드 순서가 동일하므로 변환은 완전히 일반적으로(제네릭하게)
구현 가능하다 — 메시지별 변환 코드를 따로 안 짠다.

avva_phase1.py는 `from __future__ import annotations`를 쓰므로
dataclasses.fields().type은 실제 타입이 아니라 문자열이다. 역변환(ROS -> dc)은
그 문자열을 파싱해 목적 타입을 결정한다.
"""
from __future__ import annotations

import dataclasses
import re
from enum import IntEnum

import avva_phase1 as dc


def _is_enum_name(name: str) -> bool:
    cls = getattr(dc, name, None)
    return isinstance(cls, type) and issubclass(cls, IntEnum)


def to_ros_msg(obj, ros_module):
    """dataclass 인스턴스 -> 같은 이름의 ROS 2 message 인스턴스."""
    ros_cls = getattr(ros_module, type(obj).__name__)
    ros_obj = ros_cls()
    for f in dataclasses.fields(obj):
        value = getattr(obj, f.name)
        if value is None:
            # OptionalXxx.value가 has_value=False일 때 관례적으로 None (avva_phase1.py
            # 자체는 그대로 둠). ros_obj = ros_cls()가 이미 이 필드를 rosidl 자신의
            # 정확한 기본값(uint8[16]/[32]처럼 고정길이 배열도 올바른 길이로 0-채움,
            # nested message도 재귀적으로 기본 생성)으로 채워놨으므로 그냥 안 건드리고
            # 넘어간다 — 예전엔 타입 문자열만 보고 별도로 기본값을 다시 만들었는데
            # bytes 타입에 b""(길이 0)를 넣어서 고정길이 필드에서 깨지는 버그였다.
            continue
        setattr(ros_obj, f.name, _to_ros_value(value, ros_module))
    return ros_obj


def _to_ros_value(value, ros_module):
    if dataclasses.is_dataclass(value):
        return to_ros_msg(value, ros_module)
    if isinstance(value, IntEnum):
        return int(value)
    if isinstance(value, bytes):
        # rosidl_generator_py's uint8[N] setter does numpy.array(value, dtype=uint8),
        # which mis-parses a raw `bytes` object as a numeric string rather than a
        # byte sequence — list(value) makes it an int sequence numpy accepts.
        return list(value)
    if isinstance(value, (tuple, list)):
        return [_to_ros_value(v, ros_module) for v in value]
    return value  # int/float/bool/str — rosidl_generator_py accepts these as-is


def from_ros_msg(ros_obj, dc_cls):
    """ROS 2 message 인스턴스 -> 지정한 avva_phase1 dataclass 인스턴스."""
    kwargs = {f.name: _from_ros_value(getattr(ros_obj, f.name), f.type)
              for f in dataclasses.fields(dc_cls)}
    return dc_cls(**kwargs)


def _from_ros_value(value, type_str: str):
    type_str = type_str.strip()
    if type_str == "bytes":
        return bytes(value)
    if type_str in ("int", "float", "bool", "str"):
        return {"int": int, "float": float, "bool": bool, "str": str}[type_str](value)
    m = re.fullmatch(r"list\[(.+)\]", type_str)
    if m:
        return [_from_ros_value(v, m.group(1)) for v in value]
    m = re.fullmatch(r"tuple\[(.+),\s*\.\.\.\]", type_str)
    if m:
        return tuple(_from_ros_value(v, m.group(1)) for v in value)
    if _is_enum_name(type_str):
        return getattr(dc, type_str)(int(value))
    dc_cls = getattr(dc, type_str)  # nested struct
    return from_ros_msg(value, dc_cls)
