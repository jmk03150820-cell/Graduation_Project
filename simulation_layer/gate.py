"""Simulation Layer frame pipeline: Registry + Input Coordinator + Frame Input
Gate + Snapshot + State Publisher (기준서 §7, §8.2, §9.2).

Sim frame states: WAITING_INPUTS → COLLECTING → READY → APPLYING → TICKING →
PUBLISHING → (FrameCompleteAck ACCEPTED) → WAITING_INPUTS. Any fatal error →
ABORTED, no same-epoch recovery.

Transport is a plain callable `publish(channel, msg)` (INPROC_TEST profile);
channel names follow interfaces/channels/channel_registry.yaml.
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from typing import Callable

import avva_phase1 as m
from avva_hash_v1 import control_set_digest_bytes, sha256, snapshot_bytes

from . import hashing
from .backend import RunConfig

_NO_ID = m.OptionalBoundedId(False, "")
_NO_UUID = m.OptionalUuid128(False, b"\x00" * 16)
_NO_U64 = m.OptionalUint64(False, 0)
IDENTITY_QUAT = m.Quaterniond(0.0, 0.0, 0.0, 1.0)


def _vec(v: list[float]) -> m.Vector3d:
    return m.Vector3d(v[0], v[1], v[2])


@dataclass(frozen=True)
class RegistryEntry:
    actor_id: str
    role: m.ActorRole
    owner: m.ControlOwner
    lifecycle: m.Lifecycle
    representation: m.RepresentationLevel
    class_id: m.ActorClass


@dataclass(frozen=True)
class Snapshot:
    """Immutable evaluation snapshot frozen at READY (§7.1 step 4)."""
    target_tick_id: int
    controls: dict[str, m.NeutralControl]  # APPLY-only; HOLD/NO_OP apply nothing
    snapshot_hash: bytes
    control_set_meta: m.ControlSetMeta
    completeness: m.Completeness
    validity: m.Validity


def validate_neutral_control(c: m.NeutralControl, require_steer: bool) -> int:
    """Returns 0 or a ReasonCode. §6.6 rules."""
    if c.control_mode not in (m.ControlMode.VELOCITY_TARGET, m.ControlMode.ACCELERATION_TARGET,
                              m.ControlMode.DIRECT_ACTUATION):
        return m.UNSUPPORTED_CONTROL_MODE
    mask = c.valid_fields_mask
    if mask >> 9:
        return m.INVALID_FIELD_MASK
    if require_steer and not mask & 1:
        return m.INVALID_FIELD_MASK
    required_bit = {m.ControlMode.VELOCITY_TARGET: 2, m.ControlMode.ACCELERATION_TARGET: 3}.get(c.control_mode)
    if required_bit is not None and not mask & (1 << required_bit):
        return m.INVALID_FIELD_MASK
    if c.control_mode == m.ControlMode.DIRECT_ACTUATION and not mask & 0b1100000:
        return m.INVALID_FIELD_MASK  # neither throttle nor brake present
    for bit, name in ((0, "steering_tire_angle_rad"), (1, "steering_tire_rotation_rate_rad_s"),
                      (2, "velocity_mps"), (3, "acceleration_mps2"), (4, "jerk_mps3"),
                      (5, "throttle"), (6, "brake")):
        v = getattr(c, name)
        if not mask & (1 << bit):
            if v != 0.0:
                return m.INVALID_FIELD_MASK  # masked-off values must be 0-normalized
        elif v != v or v in (float("inf"), float("-inf")):
            return m.NON_FINITE_VALUE
    # 기준서 §6.6 표34: "bit=0 값은 0-normalized하고 읽거나 hash하지 않는다" — bit7(gear)/
    # bit8(hand_brake)도 같은 규칙이지만 위 루프는 float 필드만 다뤄서 빠져 있었다.
    if not mask & (1 << 7) and c.gear != m.Gear.GEAR_UNKNOWN:
        return m.INVALID_FIELD_MASK
    if not mask & (1 << 8) and c.hand_brake:
        return m.INVALID_FIELD_MASK
    if mask & (1 << 5) and not 0.0 <= c.throttle <= 1.0:
        return m.OUT_OF_RANGE
    if mask & (1 << 6) and not 0.0 <= c.brake <= 1.0:
        return m.OUT_OF_RANGE
    if mask & (1 << 5) and mask & (1 << 6) and c.throttle > 0.0 and c.brake > 0.0:
        return m.INVALID_COMMAND_COMBINATION
    return 0


class SimulationLayer:
    def __init__(self, backend, run_id: bytes, run_epoch: int, sim_id: str,
                 registry: list[RegistryEntry], run_config: RunConfig,
                 publish: Callable[[str, object], None],
                 deadline_budget_ns: int = 50_000_000) -> None:
        # Execution Policy 경계 (Phase 2 확장점): async scheduler가 생기면 이
        # 분기에서 정책 구현을 선택한다. Phase 1은 sync lockstep 하나뿐.
        if run_config.execution_mode != m.ExecutionMode.SYNC_FIXED_STEP:
            raise NotImplementedError("Phase 1 execution policy: SYNC_FIXED_STEP only")
        self.backend = backend
        self.run_id, self.run_epoch, self.sim_id = run_id, run_epoch, sim_id
        self.run_config = run_config
        self.fixed_step_ns = run_config.fixed_step_ns
        self.deadline_budget_ns = deadline_budget_ns
        self.publish = publish
        self.registry = {e.actor_id: e for e in registry}
        self.control_set_version = 1
        self.producer_instance_id = uuid.uuid4().bytes
        self.event_seq = 0
        self.state = "WAITING_INPUTS"
        self.state_tick = 0
        self.ego_slot: m.EgoControlCommand | None = None
        self.npc_slot: m.NpcControlBatch | None = None
        self.snapshot: Snapshot | None = None
        self.decision_cache: dict[bytes, m.FrameComplete] = {}
        self.ticked: set[int] = set()  # session-scoped exactly-once guard (§7.4)
        self.events: list[dict] = []   # evidence log; JSONL 방출은 Recorder 연결 시
        self._native_frame_before = 0

    # ---- evidence --------------------------------------------------------
    def _event(self, event: str, reason_code: int = 0, header: m.CommonHeader | None = None,
              **extra) -> None:
        self.events.append({"event": event, "reason_code": reason_code,
                            "state": self.state, "state_tick": self.state_tick,
                            "wall_ns": time.time_ns(), **extra})
        # REJECT = 보낸 쪽 메시지 자체를 거부, 상태는 안 바뀜 -> 고쳐서 재전송 가능 (§2 출력 경계: RejectNotice)
        if event == "REJECT" and header is not None and reason_code:
            self._publish_reject(header, extra.get("kind", 0), reason_code, retryable=True)

    def _abort(self, reason_code: int, header: m.CommonHeader | None = None, **extra) -> None:
        # §8.6: abort evidence is flushed synchronously before anything else
        self._event("ABORT", reason_code, **extra)
        self.state = "ABORTED"
        # ABORT = 이 Run은 더 못 씀 -> 재전송으로 해결 안 됨, 새 Run 필요
        if header is not None:
            self._publish_reject(header, extra.get("kind", 0), reason_code, retryable=False)

    def _publish_reject(self, header: m.CommonHeader, rejected_kind: int, reason_code: int,
                        retryable: bool) -> None:
        notice = m.RejectNotice(
            header=None, target_component_id=header.producer_id, rejected_message_kind=rejected_kind,
            related_hash=header.payload_hash,
            tick_refs=m.TickRefs(True, self.state_tick, False, 0, False, 0, False, b"\x00" * 16),
            reason_code=reason_code, retryable=retryable,
            detail_data=m.DetailData(m.DetailKind.DETAIL_NONE, 0, 0,
                                     m.OptionalBoundedId(False, ""), m.OptionalHash256(False, b"\x00" * 32)),
            detail_message="")
        notice.header = self._header(hashing.reject_notice_hash(notice), self.state_tick * self.fixed_step_ns)
        self.publish("avva/v1/evidence/reject", notice)

    # ---- header ----------------------------------------------------------
    def _header(self, payload_hash: bytes, sim_time_ns: int,
                ego_id: str | None = None, correlation: bytes | None = None) -> m.CommonHeader:
        self.event_seq += 1
        return m.CommonHeader(
            schema_major=1, schema_minor=0, run_id=self.run_id, run_epoch=self.run_epoch,
            scope_kind=m.ScopeKind.EGO if ego_id else m.ScopeKind.SIM,
            sim_id=m.OptionalBoundedId(True, self.sim_id),
            ego_id=m.OptionalBoundedId(True, ego_id) if ego_id else _NO_ID,
            producer_id=m.ComponentId.SIM_BACKEND,
            producer_instance_id=self.producer_instance_id, event_seq=self.event_seq,
            correlation_id=m.OptionalUuid128(True, correlation) if correlation else _NO_UUID,
            sim_time_ns=m.OptionalUint64(True, sim_time_ns),
            wall_time_unix_ns=time.time_ns(), payload_hash=payload_hash)

    def _check_run(self, header: m.CommonHeader) -> int:
        if header.schema_major != 1:
            return m.SCHEMA_INCOMPATIBLE
        if header.run_id != self.run_id:
            return m.RUN_MISMATCH
        if header.run_epoch != self.run_epoch:
            return m.EPOCH_MISMATCH
        # 기준서 §5.2.1: SIM/EGO scope면 sim_id가 반드시 존재 — gate.py의 4개 핸들러는
        # 전부 SIM 또는 EGO scope 메시지만 받으므로 항상 적용된다. 예전엔 존재 여부만
        # (on_ego_control에서만) 확인했고 실제로 이 컴포넌트의 sim_id와 같은지는 어느
        # 핸들러도 검사하지 않았다 — 다른 sim_id를 향한 메시지가 그대로 통과할 수 있었음.
        if not header.sim_id.has_value or header.sim_id.value != self.sim_id:
            return m.INVALID_HEADER_SCOPE
        return 0

    # ---- registry views --------------------------------------------------
    def _expected_traffic_ids(self) -> list[str]:
        return sorted(a for a, e in self.registry.items()
                      if e.owner == m.ControlOwner.TRAFFIC_ENGINE and e.lifecycle == m.Lifecycle.ACTIVE)

    def _expected_ego_ids(self) -> list[str]:
        return sorted(a for a, e in self.registry.items()
                      if e.role == m.ActorRole.ACTOR_ROLE_EGO and e.lifecycle == m.Lifecycle.ACTIVE)

    def _control_set_meta(self) -> m.ControlSetMeta:
        rows = [(e.actor_id, e.role, e.owner, e.lifecycle, e.representation)
                for e in self.registry.values()
                if e.lifecycle == m.Lifecycle.ACTIVE
                and (e.role == m.ActorRole.ACTOR_ROLE_EGO or e.owner == m.ControlOwner.TRAFFIC_ENGINE)]
        digest = sha256(control_set_digest_bytes(rows))
        return m.ControlSetMeta(version=self.control_set_version, digest=digest,
                                expected_ego_count=len(self._expected_ego_ids()),
                                expected_traffic_count=len(self._expected_traffic_ids()))

    # ---- state publishing (bootstrap seed and post-tick) -----------------
    def bootstrap(self) -> None:
        """First authoritative state is state_tick_id=0 (§7.3)."""
        cap = self.backend.capabilities()  # capability fail-fast (§2-7): 실행 전 검증
        npc_count = len(self._expected_traffic_ids())
        if npc_count > cap.effective_max_npc:
            raise ValueError(f"scenario needs {npc_count} NPCs but "
                             f"{cap.simulator_type.name} allows max {cap.effective_max_npc}")
        self.backend.reset()
        self._native_frame_before = int(self.backend.native_frame_id())
        self._publish_state()

    def _actor_state(self, actor_id: str, states: dict | None = None) -> m.ActorState:
        e = self.registry[actor_id]
        kin = (states or self.backend.get_actor_state())[actor_id]
        dims = kin.dimensions_lwh or (4.5, 1.8, 1.5)
        return m.ActorState(
            actor_id=actor_id, actor_role=e.role, control_owner=e.owner,
            lifecycle=e.lifecycle, representation_level=e.representation, class_id=e.class_id,
            position_m=_vec(kin.position), orientation_xyzw=m.Quaterniond(*kin.orientation_xyzw),
            velocity_mps=_vec(kin.velocity), acceleration_mps2=_vec(kin.acceleration),
            angular_velocity_rad_s=_vec(kin.angular_velocity),
            dimensions_m=m.OptionalDimensions3d(True, m.Dimensions3d(*dims)),
            state_validity=m.StateValidity.STATE_VALID)

    def _native_frame_ref(self) -> m.NativeFrameRef:
        return m.NativeFrameRef(self.backend.native_session_id, self.backend.native_generation,
                                self.backend.native_frame_id())

    def _publish_state(self) -> None:
        k = self.state_tick
        sim_time = k * self.fixed_step_ns
        meta = self._control_set_meta()
        actor_ids = sorted(self.registry)
        correlation = uuid.uuid4().bytes
        states = self.backend.get_actor_state()  # snapshot 1회, actor별 재조회 금지 (§2-6)

        ws = m.WorldStateFrame(
            header=None, state_tick_id=k, native_frame_ref=self._native_frame_ref(),
            control_set_meta=meta, expected_ego_ids=self._expected_ego_ids(),
            expected_traffic_actor_ids=self._expected_traffic_ids(),
            actors=[self._actor_state(a, states) for a in actor_ids],
            backend_health=m.HealthStatus.HEALTH_OK)
        ws.header = self._header(hashing.world_state_frame_hash(ws), sim_time, correlation=correlation)
        self.publish(f"avva/v1/sim/{self.sim_id}/world_state", ws)

        cov = m.Covariance6x6(tuple(1e-4 if i % 7 == 0 else 0.0 for i in range(36)))
        for ego_id in self._expected_ego_ids():
            obs = m.EgoObservationFrame(
                header=None, state_tick_id=k, ego_state=self._actor_state(ego_id, states),
                gt_objects=[m.ObservedObject(
                    actor_id=st.actor_id, class_id=st.class_id,
                    pose=m.Pose3d(st.position_m, st.orientation_xyzw),
                    twist=m.Twist3d(st.velocity_mps, st.angular_velocity_rad_s),
                    pose_covariance=cov, twist_covariance=cov,
                    state_validity=m.StateValidity.STATE_VALID)
                    for st in (self._actor_state(a, states) for a in actor_ids if a != ego_id)],
                control_set_meta=meta, perfect_prediction_enabled=False, predicted_paths=[],
                deadline_budget_ns=self.deadline_budget_ns)
            obs.header = self._header(hashing.ego_observation_frame_hash(obs), sim_time,
                                      ego_id=ego_id, correlation=uuid.uuid4().bytes)
            self.publish(f"avva/v1/sim/{self.sim_id}/ego/{ego_id}/observation", obs)

    # ---- input coordinator (§7.1 step 2, §8.4 decision table) ------------
    def _check_ticks(self, based_on: int, target: int) -> int:
        if target != based_on + 1:
            return m.TICK_RELATION_INVALID
        expected = self.state_tick + 1
        if target < expected:
            return m.STALE_TICK
        if target > expected:
            return m.FUTURE_TICK
        return 0

    def _dedup(self, slot, msg) -> str:
        """'new' | 'duplicate' | 'conflict' for the same idempotency key."""
        if slot is None:
            return "new"
        if slot.header.payload_hash == msg.header.payload_hash:
            return "duplicate"
        return "conflict"

    def on_ego_control(self, msg: m.EgoControlCommand) -> None:
        if not msg.header.ego_id.has_value:  # sim_id는 이제 _check_run이 값까지 검사
            return self._event("REJECT", m.INVALID_HEADER_SCOPE, header=msg.header, kind=m.EGO_CONTROL_COMMAND)
        reason = self._check_run(msg.header) or self._check_ticks(msg.based_on_tick_id, msg.target_tick_id)
        if reason:
            return self._event("REJECT", reason, header=msg.header, kind=m.EGO_CONTROL_COMMAND)
        outcome = self._dedup(self.ego_slot, msg)
        if outcome == "duplicate":
            return self._event("DUPLICATE", m.DUPLICATE_IDEMPOTENT, kind=m.EGO_CONTROL_COMMAND)
        if outcome == "conflict":
            return self._abort(m.PAYLOAD_CONFLICT, header=msg.header, kind=m.EGO_CONTROL_COMMAND)
        if msg.command_status == m.CommandStatus.COMMAND_OK:
            if not msg.control.has_value:
                return self._event("REJECT", m.INVALID_COMMAND_COMBINATION, header=msg.header, kind=m.EGO_CONTROL_COMMAND)
            code = validate_neutral_control(msg.control.value, require_steer=True)
            if code:
                return self._event("REJECT", code, header=msg.header, kind=m.EGO_CONTROL_COMMAND)
        elif msg.control.has_value or msg.failure_reason == m.UNKNOWN_REASON:
            return self._event("REJECT", m.INVALID_COMMAND_COMBINATION, header=msg.header, kind=m.EGO_CONTROL_COMMAND)
        if self.state not in ("WAITING_INPUTS", "COLLECTING"):
            return self._event("REJECT", m.INVALID_STATE, header=msg.header, kind=m.EGO_CONTROL_COMMAND)
        self.ego_slot = msg
        self.state = "COLLECTING"
        self._try_ready()

    def on_npc_batch(self, msg: m.NpcControlBatch) -> None:
        reason = self._check_run(msg.header) or self._check_ticks(msg.based_on_tick_id, msg.target_tick_id)
        if reason:
            return self._event("REJECT", reason, header=msg.header, kind=m.NPC_CONTROL_BATCH)
        outcome = self._dedup(self.npc_slot, msg)
        if outcome == "duplicate":
            return self._event("DUPLICATE", m.DUPLICATE_IDEMPOTENT, kind=m.NPC_CONTROL_BATCH)
        if outcome == "conflict":
            return self._abort(m.PAYLOAD_CONFLICT, header=msg.header, kind=m.NPC_CONTROL_BATCH)
        if self.state not in ("WAITING_INPUTS", "COLLECTING"):
            return self._event("REJECT", m.INVALID_STATE, header=msg.header, kind=m.NPC_CONTROL_BATCH)
        self.npc_slot = msg
        self.state = "COLLECTING"
        self._try_ready()

    # ---- frame input gate: READY freeze + CommandSetReady (§7.1 step 4) --
    _ITEM_LEGAL = {  # (status, action, control_present, failure_nonzero) — §6.7
        (m.ItemStatus.ITEM_OK, m.CommandAction.APPLY, True, False),
        (m.ItemStatus.ITEM_OK, m.CommandAction.HOLD, False, False),
        (m.ItemStatus.ITEM_OK, m.CommandAction.NO_OP, False, False),
        (m.ItemStatus.ITEM_FAILED, m.CommandAction.COMMAND_ACTION_UNSPECIFIED, False, True),
        (m.ItemStatus.DESPAWNED, m.CommandAction.NO_OP, False, False),
    }

    def _try_ready(self) -> None:
        if self.ego_slot is None or self.npc_slot is None:
            return
        ego, npc = self.ego_slot, self.npc_slot
        target = self.state_tick + 1
        expected_traffic = self._expected_traffic_ids()
        actor_reasons: list[m.ActorReason] = []
        controls: dict[str, m.NeutralControl] = {}

        item_ids = [i.actor_id for i in npc.items]
        if item_ids != sorted(item_ids):
            first_bad = next(a for i, a in enumerate(item_ids) if i and a < item_ids[i - 1])
            actor_reasons.append(m.ActorReason(first_bad, m.ACTOR_SET_MISMATCH))
        seen: set[str] = set()
        valid_traffic = failed_traffic = 0
        for item in npc.items:
            if item.actor_id in seen:
                actor_reasons.append(m.ActorReason(item.actor_id, m.DUPLICATE_ACTOR_ID))
                failed_traffic += 1
                continue
            seen.add(item.actor_id)
            if item.actor_id not in expected_traffic:
                actor_reasons.append(m.ActorReason(item.actor_id, m.UNKNOWN_ACTOR_ID))
                failed_traffic += 1
                continue
            shape = (item.status, item.action, item.control.has_value,
                     item.failure_reason != m.UNKNOWN_REASON)
            if shape not in self._ITEM_LEGAL:
                actor_reasons.append(m.ActorReason(item.actor_id, m.INVALID_COMMAND_COMBINATION))
                failed_traffic += 1
                continue
            if item.status == m.ItemStatus.ITEM_FAILED:
                actor_reasons.append(m.ActorReason(item.actor_id, item.failure_reason))
                failed_traffic += 1
                continue
            if item.action == m.CommandAction.APPLY:
                code = validate_neutral_control(item.control.value, require_steer=True)
                if code:
                    actor_reasons.append(m.ActorReason(item.actor_id, code))
                    failed_traffic += 1
                    continue
                controls[item.actor_id] = item.control.value
            valid_traffic += 1

        missing = sorted(set(expected_traffic) - seen)
        for a in missing:
            actor_reasons.append(m.ActorReason(a, m.MISSING_ACTOR_RESPONSE))

        ego_failed = ego.command_status != m.CommandStatus.COMMAND_OK
        ego_id = ego.header.ego_id.value
        # NPC는 registry 대조(UNKNOWN_ACTOR_ID)가 있는데 ego는 없었다 — registry에
        # 없는 ego_id가 그대로 apply_control로 흘러가 backend KeyError -> 잘못
        # NATIVE_APPLY_ERROR(재시도 불가 abort)로 분류되는 문제였음. 입력 오류는
        # 여기서 잡아 재시도 가능한 CommandSetReady INVALID로 되돌린다.
        if not ego_failed and ego_id not in self._expected_ego_ids():
            actor_reasons.append(m.ActorReason(ego_id, m.UNKNOWN_ACTOR_ID))
            ego_failed = True
        if not ego_failed:
            controls[ego_id] = ego.control.value

        completeness = m.Completeness.FULL if not missing else m.Completeness.PARTIAL
        validity = (m.Validity.VALID if not ego_failed and failed_traffic == 0
                    and not any(r.reason_code in (m.DUPLICATE_ACTOR_ID, m.UNKNOWN_ACTOR_ID,
                                                  m.ACTOR_SET_MISMATCH) for r in actor_reasons)
                    else m.Validity.INVALID)

        meta = self._control_set_meta()
        # Phase 1 snapshot inputs: 1 EgoControlCommand + 1 NpcControlBatch (§4.6)
        snap_hash = sha256(snapshot_bytes(self.run_id, self.run_epoch, self.sim_id, target,
                                          meta.digest, [ego.header.payload_hash, npc.header.payload_hash]))
        self.snapshot = Snapshot(target, dict(controls), snap_hash, meta, completeness, validity)
        self.state = "READY"

        counts = m.CommandCounts(
            expected_ego=len(self._expected_ego_ids()), received_ego=1, valid_ego=0 if ego_failed else 1,
            failed_ego=1 if ego_failed else 0,
            expected_traffic=len(expected_traffic), received_traffic=len(seen),
            valid_traffic=valid_traffic, failed_traffic=failed_traffic)
        actor_reasons.sort(key=lambda r: r.actor_id)  # §6.8: "실제 정렬 report의 앞 32개"
        ready = m.CommandSetReady(
            header=None, target_tick_id=target, counts=counts, control_set_meta=meta,
            completeness=completeness, validity=validity,
            missing_actor_ids=missing[:32], actor_reasons=actor_reasons[:32],
            reports_truncated=len(missing) > 32 or len(actor_reasons) > 32,
            health_summary=m.HealthSummary(m.HealthStatus.HEALTH_OK, [], False),
            snapshot_hash=snap_hash)
        # CommandSetReady/AdvanceFrame sim_time = 근거 state k의 시간 (§5.2.1)
        ready.header = self._header(hashing.command_set_ready_hash(ready),
                                    self.state_tick * self.fixed_step_ns)
        self.publish(f"avva/v1/sim/{self.sim_id}/command_ready", ready)

    # ---- advance: 3-layer once-only defense + apply/tick (§7.4, §7.1) ----
    def on_advance_frame(self, msg: m.AdvanceFrame) -> None:
        reason = self._check_run(msg.header)
        if reason:
            return self._event("REJECT", reason, header=msg.header, kind=m.ADVANCE_FRAME)
        cached = self.decision_cache.get(msg.decision_id)
        if cached is not None:  # defense 1: idempotent resend of same decision
            self._event("DUPLICATE", m.DUPLICATE_IDEMPOTENT, kind=m.ADVANCE_FRAME)
            self.publish(f"avva/v1/sim/{self.sim_id}/frame_complete", cached)
            return
        if msg.action == m.AdvanceAction.ADVANCE_ABORT:
            # Core가 스스로 요청한 ABORT를 이행하는 것 — 이 메시지를 "거부"하는 게
            # 아니므로 RejectNotice 없음(header 미전달, 저장은 계속 위해 kind만 기록)
            return self._abort(msg.abort_reason_code or m.CORE_ABORTED, kind=m.ADVANCE_FRAME)
        if msg.action != m.AdvanceAction.ADVANCE:  # reject ADVANCE_UNSPECIFIED etc., not just non-ABORT
            return self._event("REJECT", m.OUT_OF_RANGE, header=msg.header, kind=m.ADVANCE_FRAME)
        if self.state != "READY":  # defense 2: state gate
            return self._event("REJECT", m.INVALID_STATE, header=msg.header, kind=m.ADVANCE_FRAME)
        snap = self.snapshot
        if msg.target_tick_id != snap.target_tick_id or msg.snapshot_hash != snap.snapshot_hash:
            return self._abort(m.SNAPSHOT_HASH_MISMATCH, header=msg.header, kind=m.ADVANCE_FRAME)
        if snap.target_tick_id in self.ticked:  # defense 3
            # 예전엔 bare assert였음 -> python -O로 돌리면 이 방어가 통째로 사라짐.
            # 나머지 파일 전체가 예외를 _abort()로 돌리는 것과 똑같이 처리한다.
            return self._abort(m.INTERNAL_ERROR, header=msg.header, kind=m.ADVANCE_FRAME,
                               detail="double native tick (defense 3 violated)")

        self.state = "APPLYING"
        t0 = time.perf_counter_ns()
        try:
            self.backend.apply_control(snap.controls)
        except Exception as exc:
            return self._abort(m.NATIVE_APPLY_ERROR, header=msg.header, kind=m.ADVANCE_FRAME, detail=repr(exc))
        self.state = "TICKING"
        try:
            self.backend.tick()
        except Exception as exc:
            return self._abort(m.NATIVE_TICK_ERROR, header=msg.header, kind=m.ADVANCE_FRAME, detail=repr(exc))
        self.ticked.add(snap.target_tick_id)
        native_now = int(self.backend.native_frame_id())
        if native_now != self._native_frame_before + 1:
            return self._abort(m.NATIVE_TICK_ERROR, header=msg.header, kind=m.ADVANCE_FRAME,
                               detail="native frame not exactly +1")
        self._native_frame_before = native_now
        tick_duration = time.perf_counter_ns() - t0

        self.state_tick = snap.target_tick_id
        self.state = "PUBLISHING"
        self._publish_state()  # state/observation first (§7.2)
        complete = m.FrameComplete(
            header=None, state_tick_id=self.state_tick, decision_id=msg.decision_id,
            applied_snapshot_hash=snap.snapshot_hash, native_frame_ref=self._native_frame_ref(),
            tick_duration_ns=tick_duration, control_set_meta=self._control_set_meta(), warnings=[])
        # FrameComplete sim_time = 완료된 target state k+1의 시간 (§5.2.1)
        complete.header = self._header(hashing.frame_complete_hash(complete),
                                       self.state_tick * self.fixed_step_ns)
        self.decision_cache[msg.decision_id] = complete
        self.publish(f"avva/v1/sim/{self.sim_id}/frame_complete", complete)

    def on_frame_complete_ack(self, msg: m.FrameCompleteAck) -> None:
        if self._check_run(msg.header):
            return self._event("REJECT", self._check_run(msg.header), header=msg.header, kind=m.FRAME_COMPLETE_ACK)
        if self.state != "PUBLISHING":
            return self._event("REJECT", m.INVALID_STATE, header=msg.header, kind=m.FRAME_COMPLETE_ACK)
        expected = self.decision_cache.get(msg.decision_id)
        if (expected is None or msg.state_tick_id != expected.state_tick_id
                or msg.applied_snapshot_hash != expected.applied_snapshot_hash):
            return self._abort(m.DECISION_MISMATCH, header=msg.header, kind=m.FRAME_COMPLETE_ACK)
        if msg.ack_status != m.AckStatus.ACCEPTED:
            # Core가 스스로 NACK한 것을 이행하는 것 — RejectNotice 없음(위 ADVANCE_ABORT와 동일 이유)
            return self._abort(msg.reason_code or m.CORE_ABORTED, kind=m.FRAME_COMPLETE_ACK)
        self.ego_slot = self.npc_slot = None
        self.snapshot = None
        self.state = "WAITING_INPUTS"
        self._event("FRAME_COMMITTED")
