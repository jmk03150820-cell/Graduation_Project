#!/usr/bin/env python3
"""Generate dependency-free C++, Python and fixed-layout SHM bindings.

The parser intentionally accepts only the compact IDL subset used by
schema/avva_phase1.idl. Generation fails on an unknown declaration instead of
silently dropping a field.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IDL = ROOT / "schema" / "avva_phase1.idl"


def declarations(text: str):
    text = re.sub(r"//.*", "", text)
    for m in re.finditer(r"enum\s+(\w+)\s*\{([^}]*)\}\s*;", text, re.S):
        yield "enum", m.group(1), [x.strip() for x in m.group(2).split(",") if x.strip()]
    for m in re.finditer(r"struct\s+(\w+)\s*\{([^}]*)\}\s*;", text, re.S):
        fields = []
        for raw in m.group(2).split(";"):
            raw = raw.strip()
            if not raw:
                continue
            a = re.match(r"(.+?)\s+(\w+)(?:\[(\d+)\])?$", raw)
            if not a:
                raise ValueError(f"cannot parse field: {raw!r}")
            fields.append((a.group(1).strip(), a.group(2), a.group(3)))
        yield "struct", m.group(1), fields


def constants(text: str):
    text = re.sub(r"//.*", "", text)
    result = []
    for m in re.finditer(r"const\s+(MessageKind|ReasonCode)\s+([^;]+);", text):
        for pair in m.group(2).split(","):
            name, value = pair.strip().split("=", 1)
            result.append((m.group(1), name.strip(), int(value.strip())))
    return result


CPP_SCALAR = {
    "boolean": "bool", "octet": "std::uint8_t", "unsigned short": "std::uint16_t",
    "unsigned long": "std::uint32_t", "unsigned long long": "std::uint64_t",
    "long long": "std::int64_t", "double": "double",
}


def generic(t: str, mapping: dict[str, str], seq: str) -> str:
    m = re.fullmatch(r"sequence<\s*(.+)\s*,\s*(\d+)\s*>", t)
    if m:
        return seq.format(type=generic(m.group(1), mapping, seq), n=m.group(2))
    return mapping.get(t, t)


def generate_cpp(items, consts, shm=False):
    ns = "avva::shm" if shm else "avva::msg"
    out = ["// GENERATED from schema/avva_phase1.idl. DO NOT EDIT.", "#pragma once",
           "#include <array>", "#include <cstdint>", "#include <type_traits>"]
    if not shm:
        out += ["#include <string>", "#include <vector>"]
    out += [f"namespace {ns} {{"]
    if shm:
        out += [
            "template<std::size_t N> struct FixedString { std::uint16_t length{}; std::array<std::uint8_t,N> data{}; };",
            "template<class T,std::size_t N> struct FixedSequence { std::uint32_t count{}; std::array<T,N> data{}; };",
            "using BoundedId = FixedString<63>; using BoundedText255 = FixedString<255>;",
        ]
    else:
        out += ["using BoundedId = std::string; using BoundedText255 = std::string;"]
    out += ["using Uuid128 = std::array<std::uint8_t,16>;", "using Hash256 = std::array<std::uint8_t,32>;",
            "using MessageKind = std::uint16_t; using ReasonCode = std::uint16_t;"]
    out.extend(f"inline constexpr {t} {n} = {v};" for t, n, v in consts)
    for kind, name, body in items:
        if kind == "enum":
            out.append(f"enum class {name} : std::uint8_t {{")
            out.extend(f"  {v} = {i}," for i, v in enumerate(body))
            out.append("};")
        else:
            out.append(f"struct {name} {{")
            for t, field, arr in body:
                seq = "FixedSequence<{type},{n}>" if shm else "std::vector<{type}>"
                ct = generic(t, CPP_SCALAR, seq)
                if arr:
                    ct = f"std::array<{ct},{arr}>"
                out.append(f"  {ct} {field}{{}};")
            out.append("};")
    if shm:
        for kind, name, _ in items:
            if kind == "struct":
                out.append(f"static_assert(std::is_trivially_copyable_v<{name}>);")
    out.append(f"}} // namespace {ns}")
    return "\n".join(out) + "\n"


PY_SCALAR = {
    "boolean": "bool", "octet": "int", "unsigned short": "int",
    "unsigned long": "int", "unsigned long long": "int", "long long": "int", "double": "float",
    "BoundedId": "str", "BoundedText255": "str", "Uuid128": "bytes", "Hash256": "bytes",
    "MessageKind": "int", "ReasonCode": "int",
}


def generate_python(items, consts):
    out = ["# GENERATED from schema/avva_phase1.idl. DO NOT EDIT.", "from __future__ import annotations",
           "from dataclasses import dataclass", "from enum import IntEnum", "from typing import TypeAlias", "",
           "Uuid128: TypeAlias = bytes", "Hash256: TypeAlias = bytes", "BoundedId: TypeAlias = str", "BoundedText255: TypeAlias = str", "MessageKind: TypeAlias = int", "ReasonCode: TypeAlias = int", ""]
    out.extend(f"{n}: {t} = {v}" for t, n, v in consts)
    out.append("")
    for kind, name, body in items:
        if kind == "enum":
            out.append(f"class {name}(IntEnum):")
            out.extend(f"    {v} = {i}" for i, v in enumerate(body))
            out.append("")
        else:
            out += ["@dataclass(slots=True)", f"class {name}:"]
            for t, field, arr in body:
                pt = generic(t, PY_SCALAR, "list[{type}]")
                if arr:
                    pt = f"tuple[{pt}, ...]"
                out.append(f"    {field}: {pt}")
            if not body:
                out.append("    pass")
            out.append("")
    return "\n".join(out)


def main():
    source = IDL.read_text(encoding="utf-8")
    items = list(declarations(source))
    consts = constants(source)
    (ROOT / "generated/cpp/avva_phase1.hpp").write_text(generate_cpp(items, consts), encoding="utf-8")
    (ROOT / "generated/shm/avva_phase1_shm.hpp").write_text(generate_cpp(items, consts, shm=True), encoding="utf-8")
    (ROOT / "generated/python/avva_phase1.py").write_text(generate_python(items, consts), encoding="utf-8")
    ros = ROOT / "generated/ros2/msg/AvvaPhase1.idl"
    ros.write_text(IDL.read_text(encoding="utf-8"), encoding="utf-8")


if __name__ == "__main__":
    main()
