"""Normative AVVA canonical serialization and SHA-256 helpers.

Message-specific encoders must call these primitives in IDL field order and
exclude CommonHeader. Warning/RejectNotice/StatusEvent detail_message is also
excluded by contract.
"""
from __future__ import annotations

import hashlib
import math
import struct
import uuid
from collections.abc import Callable, Iterable

PAYLOAD_PREFIX = b"AVVA-PAYLOAD-V1\0"
CONTROL_SET_PREFIX = b"AVVA-CONTROL-SET-V1\0"
SNAPSHOT_PREFIX = b"AVVA-SNAPSHOT-V1\0"


class CanonicalWriter:
    def __init__(self) -> None:
        self._data = bytearray()

    def bytes(self, value: bytes) -> "CanonicalWriter":
        self._data.extend(value); return self

    def bool8(self, value: bool) -> "CanonicalWriter":
        return self.u8(1 if value else 0)

    def u8(self, value: int) -> "CanonicalWriter":
        self._data += struct.pack("<B", value); return self

    def u16(self, value: int) -> "CanonicalWriter":
        self._data += struct.pack("<H", value); return self

    def u32(self, value: int) -> "CanonicalWriter":
        self._data += struct.pack("<I", value); return self

    def u64(self, value: int) -> "CanonicalWriter":
        self._data += struct.pack("<Q", value); return self

    def i64(self, value: int) -> "CanonicalWriter":
        self._data += struct.pack("<q", value); return self

    def f64(self, value: float) -> "CanonicalWriter":
        if not math.isfinite(value):
            raise ValueError("NaN and infinity are forbidden")
        if value == 0.0:
            value = 0.0  # normalize negative zero
        self._data += struct.pack("<d", value); return self

    def uuid128(self, value: uuid.UUID | bytes) -> "CanonicalWriter":
        raw = value.bytes if isinstance(value, uuid.UUID) else value
        if len(raw) != 16: raise ValueError("Uuid128 must be 16 bytes")
        return self.bytes(raw)  # RFC 4122 network byte order

    def hash256(self, value: bytes) -> "CanonicalWriter":
        if len(value) != 32: raise ValueError("Hash256 must be 32 bytes")
        return self.bytes(value)

    def bounded_id(self, value: str) -> "CanonicalWriter":
        raw = value.encode("utf-8")
        if not raw or len(raw) > 63: raise ValueError("BoundedId length must be 1..63 bytes")
        return self.u8(len(raw)).bytes(raw)

    def bounded_text(self, value: str) -> "CanonicalWriter":
        raw = value.encode("utf-8")
        if len(raw) > 255: raise ValueError("BoundedText255 exceeds 255 bytes")
        return self.u16(len(raw)).bytes(raw)

    def optional(self, value, encode: Callable[["CanonicalWriter", object], None]) -> "CanonicalWriter":
        self.bool8(value is not None)
        if value is not None: encode(self, value)
        return self

    def sequence(self, values: Iterable, encode: Callable[["CanonicalWriter", object], None], maximum: int) -> "CanonicalWriter":
        values = list(values)
        if len(values) > maximum: raise ValueError("bounded sequence overflow")
        self.u32(len(values))
        for value in values: encode(self, value)
        return self

    def finish(self) -> bytes:
        return bytes(self._data)


def sha256(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()


def payload_bytes(message_kind: int, encode_body: Callable[[CanonicalWriter], None]) -> bytes:
    w = CanonicalWriter().bytes(PAYLOAD_PREFIX).u16(message_kind)
    encode_body(w)
    return w.finish()


def frame_complete_ack_payload(decision_id: uuid.UUID | bytes, state_tick_id: int,
                               applied_snapshot_hash: bytes, ack_status: int,
                               reason_code: int) -> bytes:
    return payload_bytes(23, lambda w: (w.uuid128(decision_id).u64(state_tick_id)
                                        .hash256(applied_snapshot_hash).u8(ack_status)
                                        .u16(reason_code)))


def control_set_digest_bytes(rows: Iterable[tuple[str, int, int, int, int]]) -> bytes:
    rows = sorted(rows, key=lambda x: x[0].encode("utf-8"))
    w = CanonicalWriter().bytes(CONTROL_SET_PREFIX).u32(len(rows))
    for actor_id, role, owner, lifecycle, representation in rows:
        w.bounded_id(actor_id).u8(role).u8(owner).u8(lifecycle).u8(representation)
    return w.finish()


def snapshot_bytes(run_id: uuid.UUID | bytes, run_epoch: int, sim_id: str,
                   target_tick_id: int, control_set_digest: bytes,
                   input_payload_hashes: Iterable[bytes]) -> bytes:
    hashes = sorted(input_payload_hashes)
    w = (CanonicalWriter().bytes(SNAPSHOT_PREFIX).uuid128(run_id).u64(run_epoch)
         .bounded_id(sim_id).u64(target_tick_id).hash256(control_set_digest).u32(len(hashes)))
    for value in hashes: w.hash256(value)
    return w.finish()

