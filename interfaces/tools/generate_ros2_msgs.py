#!/usr/bin/env python3
"""Fix for generated/ros2/: emit real per-message .msg files instead of the
raw OMG IDL copy.

Bug found while wiring Simulation Layer to ROS 2 (2026-08-26): the existing
`generate_bindings.py::main()` writes `generated/ros2/msg/AvvaPhase1.idl` as a
byte-for-byte copy of `schema/avva_phase1.idl` (see its last 2 lines). That
file is OMG IDL with a C preprocessor `#ifndef` guard; ROS 2's rosidl IDL
parser (a restricted ROS-specific IDL profile) rejects it outright —
`colcon build --packages-select avva_interfaces` fails on `#ifndef` before
reaching a single struct. The package as distributed does not build.

This script does not change `schema/avva_phase1.idl` (the normative source,
공통 담당 전용) or any field name/type/enum ordinal/hash. It only reformats
the SAME struct/enum declarations already parsed by generate_bindings.py into
one ROS 2 .msg file per struct — the standard, robust way to author custom
ROS 2 messages (§0.4 "전송 구현": ROS 2/SHM binding choice is a team member's
freedom for a fixed logical channel). Named enum constants are not
re-embedded per field (kept as plain uint8 on the wire); the Python side
(avva_phase1.py, already IntEnum-based) remains the source of truth for
interpreting those values, so this is a representation choice, not a meaning
change. Flagged to the interfaces bundle owner as a packaging defect to fix
upstream in generate_bindings.py.

Usage: python generate_ros2_msgs.py   (run after generate_bindings.py)
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_bindings import IDL, constants, declarations  # noqa: E402

ROS_SCALAR = {
    "boolean": "bool", "octet": "uint8", "unsigned short": "uint16",
    "unsigned long": "uint32", "unsigned long long": "uint64",
    "long long": "int64", "double": "float64",
    "BoundedId": "string<=63", "BoundedText255": "string<=255",
    "Uuid128": "uint8[16]", "Hash256": "uint8[32]",
    "MessageKind": "uint16", "ReasonCode": "uint16",
}


def ros_type(t: str, enum_names: set[str]) -> str:
    m = re.fullmatch(r"sequence<\s*(.+?)\s*,\s*(\d+)\s*>", t)
    if m:
        inner = ros_type(m.group(1), enum_names)
        return f"{inner}[<={m.group(2)}]"
    if t in enum_names:
        return "uint8"  # ordinal matches IntEnum declaration order in avva_phase1.py
    return ROS_SCALAR.get(t, t)  # struct name -> bare same-package ROS 2 message reference


def generate_msg_file(name: str, fields: list[tuple[str, str, str | None]], enum_names: set[str]) -> str:
    lines = [f"# GENERATED from ../../../schema/avva_phase1.idl by generate_ros2_msgs.py. DO NOT EDIT."]
    if not fields:
        lines.append("bool _unused  # ROS 2 messages require >=1 field")
    for t, field, arr in fields:
        rt = ros_type(t, enum_names)
        if arr:
            rt = f"{rt}[{arr}]" if "[" not in rt else rt  # fixed-size array (e.g. double values[36])
        lines.append(f"{rt} {field}")
    return "\n".join(lines) + "\n"


def main() -> None:
    source = IDL.read_text(encoding="utf-8")
    items = list(declarations(source))
    enum_names = {name for kind, name, _ in items if kind == "enum"}

    msg_dir = ROOT / "generated" / "ros2" / "msg"
    for old in msg_dir.glob("*.msg"):
        old.unlink()
    (msg_dir / "AvvaPhase1.idl").unlink(missing_ok=True)  # remove the broken copy

    count = 0
    for kind, name, body in items:
        if kind != "struct":
            continue
        (msg_dir / f"{name}.msg").write_text(generate_msg_file(name, body, enum_names), encoding="utf-8")
        count += 1

    msg_names = sorted(name for kind, name, _ in items if kind == "struct")
    msg_list = "\n".join(f'  "msg/{name}.msg"' for name in msg_names)
    cmake = ROOT / "generated" / "ros2" / "CMakeLists.txt"
    cmake.write_text(
        "cmake_minimum_required(VERSION 3.8)\n"
        "project(avva_interfaces)\n"
        "find_package(ament_cmake REQUIRED)\n"
        "find_package(rosidl_default_generators REQUIRED)\n"
        f"rosidl_generate_interfaces(${{PROJECT_NAME}}\n{msg_list}\n)\n"
        "ament_export_dependencies(rosidl_default_runtime)\n"
        "ament_package()\n",
        encoding="utf-8")
    print(f"wrote {count} .msg files to {msg_dir}")


if __name__ == "__main__":
    main()
